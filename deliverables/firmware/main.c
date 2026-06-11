/*
 * ATtiny1616 ultrasonic distance meter — Etap 2 firmware (final).
 *
 * Pinout (as built, after PCB rework moving ECHO PA2 -> PB4):
 *   PA0  UPDI            programming
 *   PA3  BUTTON          input pull-up, active LOW (start measurement)
 *   PA4  LED             output, active HIGH (measurement indicator)
 *   PA5  NTC thermistor  ADC0 AIN5 (temperature compensation)
 *   PA6  GATE Q1 (BSS84) LOW = Q1 ON = op-amp (VDD_AMP) powered
 *   PB0  TX burst        40 kHz software-timed burst -> R1 -> BC817 -> TX xducer
 *   PB2  USART0 TXD      115200 8N1 (result output)
 *   PB4  ECHO            AC1 AINP3 (RX op-amp output -> comparator)
 *
 * Echo detection (zero CPU latency, fully hardware):
 *   echo -> MCP6002 -> PB4 -> AC1 (vs DAC1 threshold) -> EVSYS ASYNCCH0
 *        -> TCB0 input capture (CAPT). TCB0.CCMP = round-trip time.
 *   NOTE: PB4 = AINP3 on AC1 (positive); on AC0 it is AINN1 (negative),
 *         hence AC1 + DAC1 (AC1 uses DAC1 per datasheet 29.x).
 *   1 tick @ F_CPU = 10 MHz = 100 ns; 16-bit -> max 6.55 ms (~1.12 m).
 *
 * Power: Standby sleep + RTC/PIT periodic wake (125 ms) polling the button.
 *   Op-amp, ADC and AC are powered only during a measurement (~30 ms).
 *   Idle current dominated by RTC ULP osc (~1-3 uA) -> avg power << 2 mW.
 */

#define F_CPU 10000000UL

#include <avr/io.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdint.h>
#include <math.h>

#define BAUD_RATE        115200UL
#define USART_BAUD_VAL   ((uint16_t)((64UL * F_CPU) / (16UL * BAUD_RATE)))

/* ----- Measurement tunables ----- */
#define BURST_N_DEFAULT  8          /* coarse-ping burst cycles */
#define BLANKING_US      350        /* ring-down skip; ~6 cm floor */
#define ECHO_TIMEOUT_MS  4          /* covers ~65 cm */
#define DAC_THRESHOLD    255        /* 2.49V over 2.5V ref, just below VG */
#define MEDIAN_OF_N      9          /* shots per measurement; median of valid */
#define MIN_VALID_HITS   2          /* need >= this many valid captures */
#define MIN_TICKS        3000       /* reject ring-down (<~5 cm) */
#define MAX_TICKS        40000      /* reject implausible (>~68 cm) */

/* ----- Linear calibration:  D_mm = raw_mm * CAL_A_NUM/CAL_A_DEN + CAL_OFFSET_MM
 * Defaults are neutral (gain 1.000, offset 0). Tune against a ruler:
 * pick two reference distances, solve a and b, update these three values. */
#define CAL_A_NUM        1000
#define CAL_A_DEN        1000
#define CAL_OFFSET_MM    0

/* ----- NTC (10k, Beta model), divider: +3V3 -- NTC -- PA5 -- 10k -- GND ----- */
#define NTC_B            3950
#define NTC_R25          10000
#define NTC_R_PULLDOWN   10000
#define T_MIN_DC         (-200)
#define T_MAX_DC         (600)

/* ============================== CLOCK ================================= */
static void clock_init(void)
{
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB,
                     CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm);   /* 20M/2 = 10 MHz */
}

/* ============================== GPIO ================================== */
static void gpio_init(void)
{
    /* PA3 button: pull-up input, polled (no pin interrupt needed). */
    PORTA.DIRCLR   = PIN3_bm;
    PORTA.PIN3CTRL = PORT_PULLUPEN_bm;

    /* PA4 LED off */
    PORTA.DIRSET = PIN4_bm; PORTA.OUTCLR = PIN4_bm;

    /* PA5 NTC analog input */
    PORTA.DIRCLR   = PIN5_bm;
    PORTA.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;

    /* PA6 GATE Q1: HIGH = Q1 OFF = op-amp unpowered (idle, saves ~1 mA) */
    PORTA.DIRSET = PIN6_bm; PORTA.OUTSET = PIN6_bm;

    /* PB0 TX burst output low */
    PORTB.DIRSET = PIN0_bm; PORTB.OUTCLR = PIN0_bm;

    /* PB2 USART TX idle high */
    PORTB.DIRSET = PIN2_bm; PORTB.OUTSET = PIN2_bm;

    /* PB4 ECHO analog input to AC1 */
    PORTB.DIRCLR   = PIN4_bm;
    PORTB.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;

    /* All other pins: digital input buffer off (low power, no float current) */
    PORTA.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
    PORTA.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
    PORTA.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
    PORTA.PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc;
    PORTB.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
    PORTB.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;
    PORTB.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;
    PORTC.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
    PORTC.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
    PORTC.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
    PORTC.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;
}

static inline void opamp_on(void)  { PORTA.OUTCLR = PIN6_bm; }  /* Q1 ON  */
static inline void opamp_off(void) { PORTA.OUTSET = PIN6_bm; }  /* Q1 OFF */

/* ============================== USART ================================= */
static void usart_init(void)
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

/* ============================== ADC (NTC) ============================ */
static void adc_init(void)
{
    ADC0.CTRLC    = ADC_PRESC_DIV8_gc | ADC_REFSEL_VDDREF_gc | ADC_SAMPCAP_bm;
    ADC0.CTRLD    = ADC_INITDLY_DLY16_gc;
    ADC0.SAMPCTRL = 0;
    ADC0.CTRLA    = ADC_RESSEL_10BIT_gc;          /* enabled on demand */
}
static inline void adc_on(void)  { ADC0.CTRLA |= ADC_ENABLE_bm; }
static inline void adc_off(void) { ADC0.CTRLA &= (uint8_t)~ADC_ENABLE_bm; }

static uint16_t adc_read(uint8_t mux)
{
    ADC0.MUXPOS   = mux;
    ADC0.INTFLAGS = ADC_RESRDY_bm;
    ADC0.COMMAND  = ADC_STCONV_bm;
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) { }
    return ADC0.RES;
}

static int16_t ntc_read_temp_c16(void)
{
    uint16_t a = adc_read(ADC_MUXPOS_AIN5_gc);
    if (a == 0)    return T_MIN_DC;
    if (a >= 1023) return T_MAX_DC;

    float r     = (float)NTC_R_PULLDOWN * (float)(1023 - a) / (float)a;
    float inv_t = 1.0f / 298.15f + logf(r / (float)NTC_R25) / (float)NTC_B;
    float t_c   = 1.0f / inv_t - 273.15f;

    int32_t t_dc = (int32_t)(t_c * 10.0f + (t_c >= 0 ? 0.5f : -0.5f));
    if (t_dc < T_MIN_DC) t_dc = T_MIN_DC;
    if (t_dc > T_MAX_DC) t_dc = T_MAX_DC;
    return (int16_t)t_dc;
}

/* ============================== AC1 + DAC1 =========================== */
static void ac1_dac1_init(void)
{
    /* DAC1/AC1 reference 2.5V (must be <= VDD=3.3V). DAC1REFSEL in VREF.CTRLC,
     * DAC1REFEN in VREF.CTRLB. */
    VREF.CTRLC = (VREF.CTRLC & ~VREF_DAC1REFSEL_gm) | VREF_DAC1REFSEL_2V5_gc;
    VREF.CTRLB |= VREF_DAC1REFEN_bm;

    DAC1.DATA  = DAC_THRESHOLD;                    /* ~2.49V threshold */
    DAC1.CTRLA = DAC_ENABLE_bm;                    /* internal (no pin) */

    AC1.MUXCTRLA = AC_MUXPOS_PIN3_gc | AC_MUXNEG_DAC_gc;  /* +PB4, -DAC1 */
    AC1.CTRLA    = AC_ENABLE_bm | AC_HYSMODE_10mV_gc;
}

/* ============================== EVSYS ================================ */
static void evsys_init(void)
{
    EVSYS.ASYNCCH0   = EVSYS_ASYNCCH0_AC1_OUT_gc;      /* AC1 output  */
    EVSYS.ASYNCUSER0 = EVSYS_ASYNCUSER0_ASYNCCH0_gc;   /* -> TCB0     */
}

/* ============================== TCB0 capture ======================== */
static void tcb0_init(void)
{
    TCB0.CTRLB    = TCB_CNTMODE_CAPT_gc;
    TCB0.EVCTRL   = TCB_CAPTEI_bm;
    TCB0.INTCTRL  = 0;
    TCB0.CNT      = 0;
    TCB0.INTFLAGS = TCB_CAPT_bm;
    TCB0.CTRLA    = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
}

/* ============================== RTC / PIT (wake) ==================== */
/* Periodic Interrupt Timer wakes the CPU from Standby every ~125 ms to
 * poll the button. RTC clock = internal 32.768 kHz ULP oscillator. */
static void rtc_pit_init(void)
{
    while (RTC.STATUS & RTC_CTRLBUSY_bm) { }
    RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
    while (RTC.PITSTATUS & RTC_CTRLBUSY_bm) { }
    RTC.PITINTCTRL = RTC_PI_bm;
    RTC.PITCTRLA   = RTC_PERIOD_CYC4096_gc | RTC_PITEN_bm;  /* 4096/32768 = 125 ms */
}
ISR(RTC_PIT_vect) { RTC.PITINTFLAGS = RTC_PI_bm; }

/* ============================== TX BURST ============================ */
static void tx_burst(uint8_t n)
{
    while (n--) {
        PORTB.OUTSET = PIN0_bm; _delay_us(12);
        PORTB.OUTCLR = PIN0_bm; _delay_us(12);
    }
}

/* ============================== DISTANCE ============================ */
static uint16_t diag_ticks;

/* One ping with the hardware AC1->TCB0 capture path. Returns mm (calibrated)
 * or 0xFFFF on no valid echo. n_cycles = burst length (adaptive). */
static uint16_t measure_once_mm(int16_t t_c_dC, uint8_t n_cycles)
{
    int32_t c_x1000 = 331300L + ((int32_t)606 * (int32_t)t_c_dC) / 10L;

    PORTB.OUTCLR  = PIN0_bm;
    TCB0.CNT      = 0;
    TCB0.INTFLAGS = TCB_CAPT_bm;

    tx_burst(n_cycles);
    PORTB.OUTCLR  = PIN0_bm;

    _delay_us(BLANKING_US);
    TCB0.INTFLAGS = TCB_CAPT_bm;                   /* drop ring-down captures */

    const uint16_t timeout_ticks =
        (uint16_t)((uint32_t)F_CPU * ECHO_TIMEOUT_MS / 1000UL);
    uint16_t ticks = 0;
    while (TCB0.CNT < timeout_ticks) {
        if (TCB0.INTFLAGS & TCB_CAPT_bm) {
            uint16_t t = TCB0.CCMP;
            TCB0.INTFLAGS = TCB_CAPT_bm;
            if (t >= MIN_TICKS && t <= MAX_TICKS) { ticks = t; break; }
        }
    }
    if (ticks == 0) { diag_ticks = 0; return 0xFFFFu; }
    diag_ticks = ticks;

    /* raw distance = c[m/s] * ToF / 2 = c_x1000 * ticks / 20e6  [mm] */
    int32_t raw = (int32_t)(((int64_t)c_x1000 * (int64_t)ticks) / 20000000LL);

    /* linear calibration */
    int32_t d = (raw * CAL_A_NUM) / CAL_A_DEN + CAL_OFFSET_MM;
    if (d < 0)        d = 0;
    if (d > 0xFFFE)   d = 0xFFFE;
    return (uint16_t)d;
}

/* Adaptive burst length from a coarse distance estimate (thesis sec. 8). */
static uint8_t adaptive_n(uint16_t coarse_mm)
{
    if (coarse_mm == 0xFFFFu) return BURST_N_DEFAULT;
    if (coarse_mm < 150)      return 4;     /* near: short burst, less ring-down */
    if (coarse_mm < 250)      return 6;
    return 8;                               /* far: max SNR */
}

/* ============================== BUTTON / SLEEP ===================== */
static uint8_t button_pressed(void) { return (PORTA.IN & PIN3_bm) == 0; }

/* Standby until the PIT wakes us (~125 ms) and the button is pressed. */
static void sleep_until_button(void)
{
    for (;;) {
        set_sleep_mode(SLEEP_MODE_STANDBY);
        sleep_enable();
        sei();
        sleep_cpu();                 /* wake on PIT */
        sleep_disable();

        if (button_pressed()) {
            _delay_ms(20);           /* debounce */
            if (button_pressed()) return;
        }
    }
}

/* ============================== MAIN =============================== */
int main(void)
{
    clock_init();
    gpio_init();
    usart_init();
    adc_init();
    ac1_dac1_init();
    evsys_init();
    tcb0_init();
    rtc_pit_init();

    /* Startup blink */
    for (uint8_t i = 0; i < 3; i++) {
        PORTA.OUTSET = PIN4_bm; _delay_ms(150);
        PORTA.OUTCLR = PIN4_bm; _delay_ms(150);
    }

    uint8_t rst = RSTCTRL.RSTFR;
    RSTCTRL.RSTFR = rst;
    usart_print("=== WETI ultrasonic meter (Etap2) RSTFR=0x");
    usart_putc("0123456789ABCDEF"[(rst >> 4) & 0xF]);
    usart_putc("0123456789ABCDEF"[rst & 0xF]);
    usart_print(" ===\r\n");

    uint16_t seq = 0;

    for (;;) {
        sleep_until_button();
        seq++;

        /* Power up analog front-end for the measurement */
        opamp_on();
        adc_on();
        _delay_ms(2);                                /* op-amp settle */
        (void)adc_read(ADC_MUXPOS_AIN5_gc);          /* ADC warm-up */

        PORTA.OUTSET = PIN4_bm;                       /* LED on */

        int16_t  t_dC   = ntc_read_temp_c16();
        uint16_t coarse = measure_once_mm(t_dC, BURST_N_DEFAULT);
        uint8_t  n      = adaptive_n(coarse);

        uint16_t samples[MEDIAN_OF_N];
        uint8_t  valid = 0;
        for (uint8_t i = 0; i < MEDIAN_OF_N; i++) {
            uint16_t d = measure_once_mm(t_dC, n);
            if (d != 0xFFFFu) samples[valid++] = d;
            _delay_ms(15);
        }

        PORTA.OUTCLR = PIN4_bm;                       /* LED off */
        adc_off();
        opamp_off();                                  /* power down front-end */

        /* insertion sort -> median */
        for (uint8_t i = 1; i < valid; i++) {
            uint16_t v = samples[i]; int8_t j = i - 1;
            while (j >= 0 && samples[j] > v) { samples[j+1] = samples[j]; j--; }
            samples[j+1] = v;
        }

        usart_print("#");
        usart_print_u16(seq);
        usart_putc(' ');
        if (valid < MIN_VALID_HITS) {
            usart_print("NO ECHO");
        } else {
            usart_print("D=");
            usart_print_u16(samples[valid / 2]);
            usart_print("mm");
        }
        usart_print(" T=");
        usart_print_temp_dC(t_dC);
        usart_print("C (N=");
        usart_print_u16(n);
        usart_print(", ");
        usart_print_u16(valid);
        usart_print("/");
        usart_print_u16(MEDIAN_OF_N);
        usart_print(")\r\n");

        /* wait for release so one press = one measurement */
        while (button_pressed()) _delay_ms(10);
    }
}
