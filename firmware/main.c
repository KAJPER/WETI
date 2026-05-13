/*
 * ATtiny1616 ultrasonic distance meter - bare-metal AVR-GCC firmware.
 *
 * Pinout (matches Kacper's PCB v14):
 *   PA0  UPDI               (programming)
 *   PA1  -                  (free)
 *   PA2  ECHO     ADC0 AIN2 (analog from MCP6002 stage 2)
 *   PA3  BTN                (pull-up, active LOW)
 *   PA4  LED                (active HIGH)
 *   PA5  TEMP     ADC0 AIN5 (NTC: VDD-NTC-PA5-10k-GND)
 *   PA6  GATE               (Q1 control - Q1 not populated)
 *   PB0  BAZA TX  TCA0 WO0  (40 kHz burst -> BC817 -> nadajnik, PORTMUX alt)
 *   PB2  USART0   TXD       (115200 8N1, results)
 *
 * TCB0 free-running at F_CPU=10 MHz, 1 tick = 100 ns, 16-bit -> 6.55 ms range.
 * Echo detection: ADC0 polled in tight loop after burst, threshold crossing
 * latches TCB0.CNT in software.
 */

#define F_CPU 10000000UL

#include <avr/io.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <math.h>

/* =========================  TUNABLES  ================================== */
#define BAUD_RATE        115200UL
#define BURST_US         200        /* 8 cycles @ 40 kHz */
#define BLANKING_US      400        /* skip ring-down before listening */
#define ECHO_TIMEOUT_MS  6
#define ECHO_THRESHOLD   540        /* 0..1023, raise if false triggers */
#define DEAD_TIME_MM     0          /* tune after first measurement */
#define NTC_B            3950       /* beta, 10k NTC */
#define NTC_R25          10000
#define NTC_R_PULLDOWN   10000      /* R10 = 10k from PA5 to GND */
#define T_MIN_DC         (-200)
#define T_MAX_DC         (600)

#define USART_BAUD_VAL   ((uint16_t)((64UL * F_CPU) / (16UL * BAUD_RATE)))

/* =========================  CLOCK  ===================================== */
static void clock_init(void)
{
    /* OSC20M / 2 -> 10 MHz */
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB,
                     CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm);
}

/* =========================  GPIO  ====================================== */
static void gpio_init(void)
{
    /* PA2 ECHO analog input - disable digital input, no pull */
    PORTA.DIRCLR   = PIN2_bm;
    PORTA.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;

    /* PA3 button input with pull-up, active LOW */
    PORTA.DIRCLR   = PIN3_bm;
    PORTA.PIN3CTRL = PORT_PULLUPEN_bm;

    /* PA4 LED output, off */
    PORTA.DIRSET = PIN4_bm;
    PORTA.OUTCLR = PIN4_bm;

    /* PA5 TEMP analog input - disable digital, no pull */
    PORTA.DIRCLR   = PIN5_bm;
    PORTA.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;

    /* PA6 GATE - drive low (Q1 not populated, but keep deterministic) */
    PORTA.DIRSET = PIN6_bm;
    PORTA.OUTCLR = PIN6_bm;

    /* PB0 BAZA TX - output, start LOW (BC817 off) */
    PORTB.DIRSET = PIN0_bm;
    PORTB.OUTCLR = PIN0_bm;

    /* PB2 USART0 TXD - output, idle high */
    PORTB.DIRSET = PIN2_bm;
    PORTB.OUTSET = PIN2_bm;
}

/* =========================  USART0 (default mux: TXD=PB2)  ============= */
static void usart0_init(void)
{
    USART0.BAUD  = USART_BAUD_VAL;
    USART0.CTRLC = USART_CHSIZE_8BIT_gc;
    USART0.CTRLB = USART_TXEN_bm;
}

static void usart_putc(char c)
{
    while (!(USART0.STATUS & USART_DREIF_bm)) { }
    USART0.TXDATAL = (uint8_t)c;
}

static void usart_print(const char *s) { while (*s) usart_putc(*s++); }

static void usart_print_u16(uint16_t v)
{
    char buf[6]; int8_t i = 0;
    if (!v) { usart_putc('0'); return; }
    while (v && i < 5) { buf[i++] = (char)('0' + v % 10); v /= 10; }
    while (i--) usart_putc(buf[(uint8_t)i]);
}

static void usart_print_temp_dC(int16_t t)
{
    if (t < 0) { usart_putc('-'); t = (int16_t)-t; }
    usart_print_u16((uint16_t)(t / 10));
    usart_putc('.');
    usart_putc((char)('0' + t % 10));
}

/* =========================  TX burst — software bitbang on PB0  ======== */
/* 8 cycles of 40 kHz = 200 us. Half period = 12.5 us = 125 CPU cycles.    */
static void tca0_init(void) { /* nothing — software bitbang */ }

static inline void tca0_burst_on(void) { /* unused */ }
static inline void tca0_burst_off(void) { PORTB.OUTCLR = PIN0_bm; }

static void tx_burst_8cycles(void)
{
    for (uint8_t i = 0; i < 8; i++) {
        PORTB.OUTSET = PIN0_bm;
        _delay_us(12);
        PORTB.OUTCLR = PIN0_bm;
        _delay_us(12);
    }
}

/* =========================  TCB0 free-running counter  ================= */
static void tcb0_init(void)
{
    TCB0.CTRLB   = TCB_CNTMODE_INT_gc;       /* periodic int / free-run */
    TCB0.EVCTRL  = 0;
    TCB0.INTCTRL = 0;
    TCB0.CCMP    = 0xFFFF;
    TCB0.CNT     = 0;
    TCB0.CTRLA   = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
}

/* =========================  ADC0  ====================================== */
/* Fast 10-bit, VDD ref. ADC clock = F_CPU/8 = 1.25 MHz, 13 ADC cycles      */
/* per conversion = ~10.4 us per sample. Sample-to-sample ~ 11 us.         */
static void adc0_init(void)
{
    ADC0.CTRLC    = ADC_PRESC_DIV8_gc
                  | ADC_REFSEL_VDDREF_gc
                  | ADC_SAMPCAP_bm;
    ADC0.CTRLA    = ADC_RESSEL_10BIT_gc | ADC_ENABLE_bm;
    ADC0.CTRLD    = ADC_INITDLY_DLY16_gc;
    ADC0.SAMPCTRL = 0;
}

static uint16_t adc_read_blocking(uint8_t mux)
{
    ADC0.MUXPOS  = mux;
    ADC0.INTFLAGS = ADC_RESRDY_bm;
    ADC0.COMMAND = ADC_STCONV_bm;
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) { }
    return ADC0.RES;
}

/* =========================  NTC -> temp (tenths degC)  ================= */
/* Topology: +3V3 -- NTC -- PA5 -- R10(10k) -- GND
 * V_PA5 = V+ * R10 / (R_NTC + R10)
 * R_NTC = R10 * (1023 - adc) / adc                                         */
static int16_t ntc_read_temp_c16(void)
{
    uint16_t adc = adc_read_blocking(ADC_MUXPOS_AIN5_gc);
    if (adc == 0)    return T_MIN_DC;            /* NTC open / no current */
    if (adc >= 1023) return T_MAX_DC;            /* NTC short / very hot  */

    float r     = (float)NTC_R_PULLDOWN * (float)(1023 - adc) / (float)adc;
    float inv_t = 1.0f / 298.15f
                + logf(r / (float)NTC_R25) / (float)NTC_B;
    float t_c   = 1.0f / inv_t - 273.15f;

    int32_t t_dc = (int32_t)(t_c * 10.0f + (t_c >= 0 ? 0.5f : -0.5f));
    if (t_dc < T_MIN_DC) t_dc = T_MIN_DC;
    if (t_dc > T_MAX_DC) t_dc = T_MAX_DC;
    return (int16_t)t_dc;
}

/* Diagnostic data filled by measure_distance_mm() */
static uint16_t diag_adc_min;
static uint16_t diag_adc_max;
static uint16_t diag_adc_baseline;
static uint16_t diag_samples;

/* =========================  Distance measurement  ====================== */
static uint16_t measure_distance_mm(int16_t t_c_dC)
{
    int32_t c_x1000 = 331300L + ((int32_t)606 * (int32_t)t_c_dC) / 10L;

    ADC0.MUXPOS = ADC_MUXPOS_AIN2_gc;

    /* Read baseline (no TX active) */
    diag_adc_baseline = adc_read_blocking(ADC_MUXPOS_AIN2_gc);

    PORTB.OUTCLR = PIN0_bm;

    TCB0.CNT = 0;

    /* 8-cycle 40 kHz burst via software bitbang on PB0 */
    tx_burst_8cycles();
    PORTB.OUTCLR = PIN0_bm;

    _delay_us(BLANKING_US);

    uint16_t echo_ticks = 0;
    uint16_t mn = 0xFFFF, mx = 0, n = 0;
    const uint16_t timeout_ticks = (uint16_t)((uint32_t)F_CPU
                                  * ECHO_TIMEOUT_MS / 1000UL);

    ADC0.INTFLAGS = ADC_RESRDY_bm;
    ADC0.COMMAND  = ADC_STCONV_bm;

    while (TCB0.CNT < timeout_ticks) {
        if (ADC0.INTFLAGS & ADC_RESRDY_bm) {
            uint16_t sample = ADC0.RES;
            if (sample < mn) mn = sample;
            if (sample > mx) mx = sample;
            n++;
            if (sample >= ECHO_THRESHOLD && echo_ticks == 0) {
                echo_ticks = TCB0.CNT;
                /* keep sampling to fill min/max for diagnostics */
            }
            ADC0.INTFLAGS = ADC_RESRDY_bm;
            ADC0.COMMAND  = ADC_STCONV_bm;
        }
    }

    diag_adc_min = mn;
    diag_adc_max = mx;
    diag_samples = n;

    if (echo_ticks == 0) return 0xFFFFu;

    int32_t dist = (int32_t)(((int64_t)c_x1000 * (int64_t)echo_ticks)
                              / 20000000LL);

    if (dist <= DEAD_TIME_MM) return 0;
    dist -= DEAD_TIME_MM;
    if (dist > 0xFFFE) dist = 0xFFFE;
    return (uint16_t)dist;
}

/* =========================  Button (PA3, active LOW)  ================== */
static uint8_t button_pressed(void) { return (PORTA.IN & PIN3_bm) == 0; }

static void wait_button_press(void)
{
    while (button_pressed()) _delay_ms(5);
    _delay_ms(20);
    for (;;) {
        if (button_pressed()) {
            _delay_ms(20);
            if (button_pressed()) return;
        }
        _delay_ms(5);
    }
}

/* =========================  MAIN  ====================================== */
int main(void)
{
    clock_init();
    gpio_init();
    usart0_init();
    adc0_init();
    tca0_init();
    tcb0_init();

    cli();

    /* Startup blink - 3x */
    for (uint8_t i = 0; i < 3; i++) {
        PORTA.OUTSET = PIN4_bm; _delay_ms(200);
        PORTA.OUTCLR = PIN4_bm; _delay_ms(200);
    }

    /* TX TEST: continuous 40 kHz on PB0 for 5 s */
    PORTA.OUTSET = PIN4_bm;
    usart_print("TX TEST: 5s 40kHz on PB0\r\n");
    for (uint32_t i = 0; i < 200000UL; i++) {
        PORTB.OUTSET = PIN0_bm; _delay_us(12);
        PORTB.OUTCLR = PIN0_bm; _delay_us(12);
    }
    PORTA.OUTCLR = PIN4_bm;

    usart_print("ATtiny1616 ultrasonic meter ready\r\n");

    while (1) {
        wait_button_press();

        PORTA.OUTSET = PIN4_bm;

        int16_t  t_dC = ntc_read_temp_c16();
        uint16_t d_mm = measure_distance_mm(t_dC);

        PORTA.OUTCLR = PIN4_bm;

        if (d_mm == 0xFFFFu) {
            usart_print("NO ECHO T=");
            usart_print_temp_dC(t_dC);
            usart_print("C");
        } else {
            usart_print("D=");
            usart_print_u16(d_mm);
            usart_print("mm T=");
            usart_print_temp_dC(t_dC);
            usart_print("C");
        }
        /* Diagnostic: baseline ADC, min/max during listen, sample count */
        usart_print(" [base=");
        usart_print_u16(diag_adc_baseline);
        usart_print(" min=");
        usart_print_u16(diag_adc_min);
        usart_print(" max=");
        usart_print_u16(diag_adc_max);
        usart_print(" n=");
        usart_print_u16(diag_samples);
        usart_print("]\r\n");
    }
}
