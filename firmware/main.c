/*
 * ATtiny1616 ultrasonic distance meter - working version.
 *
 * Pinout: PA0 UPDI, PA2 ECHO(ADC AIN2), PA3 BTN, PA4 LED, PA5 NTC,
 *         PA6 GATE(Q1: LOW=ON, powers VDD_AMP), PB0 BAZA TX, PB2 USART TXD.
 *
 * Q1 (BSS84 P-MOSFET) connects +3V3 to VDD_AMP when ON.
 * PA6 must be LOW to enable op-amp. Keep it LOW always.
 */

#define F_CPU 10000000UL

#include <avr/io.h>
#include <avr/cpufunc.h>
#include <util/delay.h>
#include <stdint.h>
#include <math.h>

#define BAUD_RATE        115200UL
#define USART_BAUD_VAL   ((uint16_t)((64UL * F_CPU) / (16UL * BAUD_RATE)))

#define BURST_CYCLES     16         /* TX strength */
#define BLANKING_US      500        /* allows ~10cm minimum */
#define ECHO_TIMEOUT_MS  4          /* covers ~65cm */
#define ECHO_BASELINE    846        /* nominal VG ADC reading */
#define ECHO_MARGIN      35         /* deviation threshold for sample */
#define SUSTAIN_K        4          /* require K consecutive samples above */
#define MEDIAN_OF_N      5          /* odd N for clean median */

/* Linear calibration: real_mm = reading_mm * CAL_SCALE / 1000 + CAL_OFFSET
 * Tune from measurements at known distances. */
#define CAL_SCALE        1500       /* 1000 = no scaling; >1000 stretches */
#define CAL_OFFSET       0
#define NTC_B            3950
#define NTC_R25          10000
#define NTC_R_PULLDOWN   10000
#define T_MIN_DC         (-200)
#define T_MAX_DC         (600)

/* ============================== INIT ================================== */
static void clock_init(void)
{
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB,
                     CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm);
}

static void gpio_init(void)
{
    PORTA.DIRCLR   = PIN2_bm;
    PORTA.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;          /* PA2 analog */
    PORTA.DIRCLR   = PIN3_bm;
    PORTA.PIN3CTRL = PORT_PULLUPEN_bm;                   /* PA3 button */
    PORTA.DIRSET = PIN4_bm; PORTA.OUTCLR = PIN4_bm;      /* PA4 LED off */
    PORTA.DIRCLR   = PIN5_bm;
    PORTA.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;          /* PA5 NTC analog */
    PORTA.DIRSET = PIN6_bm; PORTA.OUTCLR = PIN6_bm;      /* PA6 GATE LOW = Q1 ON */
    PORTB.DIRSET = PIN0_bm; PORTB.OUTCLR = PIN0_bm;      /* PB0 TX */
    PORTB.DIRSET = PIN2_bm; PORTB.OUTSET = PIN2_bm;      /* PB2 USART TX */
}

static void usart_init(void)
{
    USART0.BAUD  = USART_BAUD_VAL;
    USART0.CTRLC = USART_CHSIZE_8BIT_gc;
    USART0.CTRLB = USART_TXEN_bm;
}

static void adc_init(void)
{
    ADC0.CTRLC    = ADC_PRESC_DIV8_gc | ADC_REFSEL_VDDREF_gc | ADC_SAMPCAP_bm;
    ADC0.CTRLA    = ADC_RESSEL_10BIT_gc | ADC_ENABLE_bm;
    ADC0.CTRLD    = ADC_INITDLY_DLY16_gc;
    ADC0.SAMPCTRL = 0;
}

/* ============================== USART ================================= */
static void usart_putc(char c)
{
    while (!(USART0.STATUS & USART_DREIF_bm)) { }
    USART0.TXDATAL = (uint8_t)c;
}

static void usart_print(const char *s) { while (*s) usart_putc(*s++); }

static void usart_print_u16(uint16_t v)
{
    char b[6]; int8_t i = 0;
    if (!v) { usart_putc('0'); return; }
    while (v && i < 5) { b[i++] = '0' + v % 10; v /= 10; }
    while (i--) usart_putc(b[(uint8_t)i]);
}

static void usart_print_temp_dC(int16_t t)
{
    if (t < 0) { usart_putc('-'); t = (int16_t)-t; }
    usart_print_u16((uint16_t)(t / 10));
    usart_putc('.');
    usart_putc((char)('0' + t % 10));
}

/* ============================== ADC =================================== */
static uint16_t adc_read(uint8_t mux)
{
    ADC0.MUXPOS  = mux;
    ADC0.INTFLAGS = ADC_RESRDY_bm;
    ADC0.COMMAND  = ADC_STCONV_bm;
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) { }
    return ADC0.RES;
}

/* ============================== NTC =================================== */
/* Topology: +3V3 -- NTC -- PA5 -- R10(10k) -- GND
 * R_NTC = R10 * (1023 - adc) / adc */
static int16_t ntc_read_temp_c16(void)
{
    uint16_t adc = adc_read(ADC_MUXPOS_AIN5_gc);
    if (adc == 0)    return T_MIN_DC;
    if (adc >= 1023) return T_MAX_DC;

    float r     = (float)NTC_R_PULLDOWN * (float)(1023 - adc) / (float)adc;
    float inv_t = 1.0f / 298.15f + logf(r / (float)NTC_R25) / (float)NTC_B;
    float t_c   = 1.0f / inv_t - 273.15f;

    int32_t t_dc = (int32_t)(t_c * 10.0f + (t_c >= 0 ? 0.5f : -0.5f));
    if (t_dc < T_MIN_DC) t_dc = T_MIN_DC;
    if (t_dc > T_MAX_DC) t_dc = T_MAX_DC;
    return (int16_t)t_dc;
}

/* ============================== TX BURST ============================== */
static void tx_burst(void)
{
    for (uint8_t i = 0; i < BURST_CYCLES; i++) {
        PORTB.OUTSET = PIN0_bm; _delay_us(12);
        PORTB.OUTCLR = PIN0_bm; _delay_us(12);
    }
}

/* ============================== DISTANCE ============================== */
static uint16_t diag_baseline, diag_peak, diag_peak_us, diag_max, diag_min;

static uint16_t measure_distance_mm(int16_t t_c_dC)
{
    int32_t c_x1000 = 331300L + ((int32_t)606 * (int32_t)t_c_dC) / 10L;

    /* Baseline reading before TX */
    diag_baseline = adc_read(ADC_MUXPOS_AIN2_gc);

    PORTB.OUTCLR = PIN0_bm;
    tx_burst();
    PORTB.OUTCLR = PIN0_bm;

    /* Wait for ring-down to settle */
    _delay_us(BLANKING_US);

    /* Find FIRST time K consecutive samples deviate above ECHO_MARGIN.
     * That filters out single-sample ringdown spikes; real echo has
     * sustained energy (40kHz burst = 3 samples/cycle at ~125kHz). */
    ADC0.MUXPOS = ADC_MUXPOS_AIN2_gc;
    uint16_t echo_us = 0;
    uint8_t  sustain = 0;
    uint16_t peak_dev = 0;
    uint16_t peak_us  = 0;
    uint16_t mx = 0, mn = 0xFFFF;
    const uint16_t total_us = ECHO_TIMEOUT_MS * 1000;
    uint16_t elapsed_us = BLANKING_US;

    while (elapsed_us < total_us) {
        ADC0.INTFLAGS = ADC_RESRDY_bm;
        ADC0.COMMAND  = ADC_STCONV_bm;
        while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) { }
        uint16_t s = ADC0.RES;
        if (s > mx) mx = s;
        if (s < mn) mn = s;
        uint16_t dev = (s > diag_baseline) ? (s - diag_baseline)
                                            : (diag_baseline - s);
        if (dev > peak_dev) {
            peak_dev = dev;
            peak_us  = elapsed_us;
        }
        if (echo_us == 0) {
            if (dev >= ECHO_MARGIN) {
                sustain++;
                if (sustain >= SUSTAIN_K) {
                    /* Echo starts SUSTAIN_K samples earlier */
                    echo_us = elapsed_us - (uint16_t)(SUSTAIN_K - 1) * 8;
                }
            } else {
                sustain = 0;
            }
        }
        elapsed_us += 8;
    }

    diag_peak    = peak_dev;
    diag_peak_us = peak_us;
    diag_max     = mx;
    diag_min     = mn;

    if (echo_us == 0) return 0xFFFFu;

    /* distance_mm = c_x1000 * echo_us / 2000000 */
    int32_t raw_mm = (int32_t)(((int64_t)c_x1000 * (int64_t)echo_us)
                                / 2000000LL);

    /* Linear calibration */
    int32_t cal_mm = (raw_mm * CAL_SCALE) / 1000 + CAL_OFFSET;

    if (cal_mm <= 0) return 1;
    if (cal_mm > 0xFFFE) cal_mm = 0xFFFE;
    return (uint16_t)cal_mm;
}

/* ============================== BUTTON ================================ */
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

/* =============================== MAIN ================================= */
int main(void)
{
    clock_init();
    gpio_init();
    usart_init();
    adc_init();

    for (uint8_t i = 0; i < 4; i++) (void)adc_read(ADC_MUXPOS_AIN2_gc);

    /* Startup blink */
    for (uint8_t i = 0; i < 3; i++) {
        PORTA.OUTSET = PIN4_bm; _delay_ms(150);
        PORTA.OUTCLR = PIN4_bm; _delay_ms(150);
    }

    /* Print reset cause so we can detect brown-outs */
    uint8_t rst = RSTCTRL.RSTFR;
    RSTCTRL.RSTFR = rst;  /* clear flags */
    usart_print("=== BOOT (RSTFR=0x");
    usart_putc("0123456789ABCDEF"[(rst >> 4) & 0xF]);
    usart_putc("0123456789ABCDEF"[rst & 0xF]);
    usart_print(") ===\r\n");

    uint16_t seq = 0;

    while (1) {
        wait_button_press();
        seq++;

        PORTA.OUTSET = PIN4_bm;

        int16_t t_dC = ntc_read_temp_c16();

        /* Take MEDIAN_OF_N measurements, sort, pick middle. Filters noise. */
        uint16_t samples[MEDIAN_OF_N];
        uint8_t  valid = 0;
        for (uint8_t i = 0; i < MEDIAN_OF_N; i++) {
            uint16_t d = measure_distance_mm(t_dC);
            if (d != 0xFFFFu) samples[valid++] = d;
            _delay_ms(15);                  /* spacing between bursts */
        }

        PORTA.OUTCLR = PIN4_bm;

        /* Simple insertion sort */
        for (uint8_t i = 1; i < valid; i++) {
            uint16_t v = samples[i]; int8_t j = i - 1;
            while (j >= 0 && samples[j] > v) { samples[j+1] = samples[j]; j--; }
            samples[j+1] = v;
        }

        usart_print("#");
        usart_print_u16(seq);
        usart_putc(' ');
        if (valid == 0) {
            usart_print("NO ECHO");
        } else {
            uint16_t d_mm = samples[valid / 2];   /* median */
            usart_print("D=");
            usart_print_u16(d_mm);
            usart_print("mm");
        }
        usart_print(" T=");
        usart_print_temp_dC(t_dC);
        usart_print("C  (");
        usart_print_u16(valid);
        usart_print("/");
        usart_print_u16(MEDIAN_OF_N);
        usart_print(" hits, lastpeak=");
        usart_print_u16(diag_peak);
        usart_print("@");
        usart_print_u16(diag_peak_us);
        usart_print("us)\r\n");
    }
}
