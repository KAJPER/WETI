/*
 * ATtiny1616 ultrasonic distance meter — v2 hardware echo capture.
 *
 * Pinout (after PCB mod):
 *   PA0  UPDI
 *   PA1  -                (free, was wrongly assumed ECHO earlier)
 *   PA2  -                (free, was ADC ECHO before PCB mod)
 *   PA3  BTN              (pull-up, active LOW, BOTHEDGES wake)
 *   PA4  LED              (active HIGH)
 *   PA5  TEMP NTC         (ADC0 AIN5)
 *   PA6  GATE Q1          (LOW = Q1 ON = op-amp powered)
 *   PB0  BAZA TX          (40 kHz software bitbang)
 *   PB2  USART0 TXD       (115200 8N1)
 *   PB4  ECHO             (AC1 AINP3 — hardware capture path)
 *
 * Detection chain (zero CPU latency):
 *   echo → op-amp → PB4 → AC1 (vs DAC1 threshold) → EVSYS → TCB0.CAPT
 *   NOTE: PB4 is AINP3 on AC1 (positive); on AC0 it is AINN1 (negative).
 * TCB0 latches CCMP exactly when AC0 toggles, no software loop needed.
 * 1 tick @ F_CPU=10 MHz = 100 ns; 16-bit counter = max 6.55 ms = 1.12 m.
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

#define BURST_CYCLES     12         /* burst strength (300us, ok for 10cm) */
#define BLANKING_US      350        /* min detection ~6cm; ring-down skip */
#define ECHO_TIMEOUT_MS  4          /* covers ~65 cm */
/* DAC ref MUST be <= VDD. At 3.3V only 0.55/1.1/1.5/2.5V are valid.
 * Use 2.5V ref. Threshold = 255/256*2.5 = 2.49V, sits ~0.20V below VG=2.69V.
 * Quiescent: PB4(2.69V) > DAC1(2.49V) → AC1 HIGH (stable, no false edges).
 * Echo: signal dips below 2.49V → AC1 toggles → TCB0 captures rising edge. */
#define DAC_THRESHOLD    255        /* 2.49V over 2.5V ref, just below VG */
#define MEDIAN_OF_N      9          /* more shots, median of valid ones */
#define MIN_VALID_HITS   2          /* need >=N valid captures to report */
#define MIN_TICKS        3000       /* reject ring-down (<~5cm) false captures */
#define MAX_TICKS        40000      /* reject implausible (>~68cm) */

/* Calibration vs ruler (refit after hardware switch). Start neutral. */
#define DEAD_TIME_MM     0          /* tune from first measurements */
#define CAL_TEMP_C       25

#define NTC_B            3950
#define NTC_R25          10000
#define NTC_R_PULLDOWN   10000
#define T_MIN_DC         (-200)
#define T_MAX_DC         (600)

/* ============================== CLOCK ================================= */
static void clock_init(void)
{
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB,
                     CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm);
}

/* ============================== GPIO ================================== */
static void gpio_init(void)
{
    /* PA3 button with pull-up + BOTHEDGES IRQ (IDLE wake) */
    PORTA.DIRCLR   = PIN3_bm;
    PORTA.PIN3CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;

    /* PA4 LED */
    PORTA.DIRSET = PIN4_bm; PORTA.OUTCLR = PIN4_bm;

    /* PA5 NTC analog input */
    PORTA.DIRCLR   = PIN5_bm;
    PORTA.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;

    /* PA6 GATE Q1 — DIAGNOSTIC: Q1 ALWAYS ON so PB4 can be measured stably */
    PORTA.DIRSET = PIN6_bm; PORTA.OUTCLR = PIN6_bm;

    /* PB0 TX burst output */
    PORTB.DIRSET = PIN0_bm; PORTB.OUTCLR = PIN0_bm;

    /* PB2 USART TX */
    PORTB.DIRSET = PIN2_bm; PORTB.OUTSET = PIN2_bm;

    /* PB4 ECHO input for AC0 — disable digital, no pull */
    PORTB.DIRCLR   = PIN4_bm;
    PORTB.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;

    /* Other unused pins: disable digital input (low power) */
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

static inline void opamp_on(void)  { PORTA.OUTCLR = PIN6_bm; }
static inline void opamp_off(void) { PORTA.OUTSET = PIN6_bm; }

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

/* ============================== ADC (NTC only) ======================== */
static void adc_init(void)
{
    ADC0.CTRLC    = ADC_PRESC_DIV8_gc | ADC_REFSEL_VDDREF_gc | ADC_SAMPCAP_bm;
    ADC0.CTRLD    = ADC_INITDLY_DLY16_gc;
    ADC0.SAMPCTRL = 0;
    ADC0.CTRLA    = ADC_RESSEL_10BIT_gc;  /* enable on demand */
}
static inline void adc_on(void)  { ADC0.CTRLA |= ADC_ENABLE_bm; }
static inline void adc_off(void) { ADC0.CTRLA &= (uint8_t)~ADC_ENABLE_bm; }

static uint16_t adc_read(uint8_t mux)
{
    ADC0.MUXPOS  = mux;
    ADC0.INTFLAGS = ADC_RESRDY_bm;
    ADC0.COMMAND  = ADC_STCONV_bm;
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) { }
    return ADC0.RES;
}

/* ============================== NTC =================================== */
/* Topology: +3V3 -- NTC -- PA5 -- R10(10k) -- GND */
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

/* ============================== AC0 + DAC0 ============================ */
/* DAC0 generates threshold internally (no pin output).
 * CRITICAL (verified datasheet Table 5-1): on AC0, PB4 = AINN1 (NEGATIVE);
 * PB4 = AINP3 (positive) ONLY on AC1. So we MUST use AC1 for echo on PB4.
 * Datasheet 29.x: "AC1 uses DAC1" — threshold comes from DAC1, ref via
 * VREF.CTRLC (DAC1REFSEL) + VREF.CTRLB (DAC1REFEN, bit3).
 * AC1 compares PB4 (AINP3) vs DAC1; AC1_OUT → EVSYS → TCB0 capture. */
static void ac1_dac1_init(void)
{
    /* DAC1/AC1 reference: 2.5V internal — MUST be <= VDD (3.3V).
     * DAC1REFSEL lives in VREF.CTRLC; DAC1REFEN in VREF.CTRLB. */
    VREF.CTRLC = (VREF.CTRLC & ~VREF_DAC1REFSEL_gm) | VREF_DAC1REFSEL_2V5_gc;
    VREF.CTRLB |= VREF_DAC1REFEN_bm;

    DAC1.DATA  = DAC_THRESHOLD;          /* 255 → ~2.49V, just below VG=2.69V */
    DAC1.CTRLA = DAC_ENABLE_bm;          /* internal only (DAC1 has no pin) */

    AC1.MUXCTRLA = AC_MUXPOS_PIN3_gc      /* + input = AINP3 = PB4 (AC1!) */
                 | AC_MUXNEG_DAC_gc;      /* - input = DAC1 threshold */
    AC1.CTRLA    = AC_ENABLE_bm           /* event-only output (OUTEN=0) */
                 | AC_HYSMODE_10mV_gc;    /* min hysteresis = max sensitivity */
}

/* ============================== EVSYS ================================= */
static void evsys_init(void)
{
    EVSYS.ASYNCCH0   = EVSYS_ASYNCCH0_AC1_OUT_gc;      /* source = AC1 output */
    EVSYS.ASYNCUSER0 = EVSYS_ASYNCUSER0_ASYNCCH0_gc;   /* TCB0 = async user 0 */
}

/* ============================== TCB0 input capture =================== */
static void tcb0_init(void)
{
    TCB0.CTRLB   = TCB_CNTMODE_CAPT_gc;            /* capture on event edge */
    TCB0.EVCTRL  = TCB_CAPTEI_bm;                  /* event input enable */
    TCB0.INTCTRL = 0;                              /* polled, no interrupt */
    TCB0.CNT     = 0;
    TCB0.INTFLAGS = TCB_CAPT_bm;
    TCB0.CTRLA   = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
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
static uint16_t diag_ticks;            /* last raw TCB0 ticks captured */
static uint8_t  diag_captured;         /* 1 if echo detected */

/* Hardware echo detection: AC0 → EVSYS → TCB0.CCMP.
 * Workflow:
 *   1. Reset TCB0.CNT = 0 right before TX burst
 *   2. Burst fires (200 µs)
 *   3. Wait BLANKING_US — capture flag may set from ring-down (discard)
 *   4. Clear INTFLAGS to discard ring-down captures
 *   5. Wait for next CAPT (real echo) up to TIMEOUT
 *   6. Read CCMP = round-trip time in 100 ns ticks
 */
static uint16_t measure_distance_mm(int16_t t_c_dC)
{
    int32_t c_x1000 = 331300L + ((int32_t)606 * (int32_t)t_c_dC) / 10L;

    PORTB.OUTCLR  = PIN0_bm;
    TCB0.CNT      = 0;
    TCB0.INTFLAGS = TCB_CAPT_bm;     /* clear any stale capture */

    tx_burst();
    PORTB.OUTCLR  = PIN0_bm;

    /* DIAGNOSTIC: capture during burst (ring-down). Save first ring-down time. */
    uint16_t ringdown_ticks = 0;
    if (TCB0.INTFLAGS & TCB_CAPT_bm) {
        ringdown_ticks = TCB0.CCMP;
    }

    /* Blanking — let ring-down decay */
    _delay_us(BLANKING_US);

    /* Discard captures that happened during burst/blanking */
    if (TCB0.INTFLAGS & TCB_CAPT_bm) {
        ringdown_ticks = TCB0.CCMP;
        TCB0.INTFLAGS = TCB_CAPT_bm;
    }

    /* Wait for a VALID echo capture (ticks in [MIN_TICKS, MAX_TICKS]) or
     * timeout. Captures below MIN_TICKS are ring-down tail → skip them. */
    const uint16_t timeout_ticks = (uint16_t)((uint32_t)F_CPU * ECHO_TIMEOUT_MS / 1000UL);
    uint16_t ticks = 0;
    while (TCB0.CNT < timeout_ticks) {
        if (TCB0.INTFLAGS & TCB_CAPT_bm) {
            uint16_t t = TCB0.CCMP;
            TCB0.INTFLAGS = TCB_CAPT_bm;
            if (t >= MIN_TICKS) { ticks = t; break; }   /* valid echo */
            ringdown_ticks = t;                         /* too early, keep waiting */
        }
    }

    if (ticks == 0) {
        diag_captured = 0;
        diag_ticks    = ringdown_ticks;
        return 0xFFFFu;
    }

    diag_captured = 1;
    diag_ticks    = ticks;

    /* distance_mm = c[m/s] * ToF / 2 = c_x1000 * ticks * 100e-9 / 2 * 1000
     *             = c_x1000 * ticks / 20_000_000 */
    int32_t dist = (int32_t)(((int64_t)c_x1000 * (int64_t)ticks) / 20000000LL);

    if (dist <= DEAD_TIME_MM) return 1;
    dist -= DEAD_TIME_MM;
    if (dist > 0xFFFE) dist = 0xFFFE;
    return (uint16_t)dist;
}

/* ============================== BUTTON / SLEEP ======================== */
static uint8_t button_pressed(void) { return (PORTA.IN & PIN3_bm) == 0; }

ISR(PORTA_PORT_vect) { PORTA.INTFLAGS = PORTA.INTFLAGS; }

static void sleep_until_button(void)
{
    while (button_pressed()) _delay_ms(5);
    _delay_ms(20);
    for (;;) {
        cli();
        if (button_pressed()) {
            sei();
            _delay_ms(20);
            if (button_pressed()) return;
            continue;
        }
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_enable();
        sei();
        sleep_cpu();
        sleep_disable();
    }
}

/* =============================== MAIN ================================= */
int main(void)
{
    clock_init();
    gpio_init();
    usart_init();
    adc_init();
    ac1_dac1_init();
    evsys_init();
    tcb0_init();

    /* Startup blink */
    for (uint8_t i = 0; i < 3; i++) {
        PORTA.OUTSET = PIN4_bm; _delay_ms(150);
        PORTA.OUTCLR = PIN4_bm; _delay_ms(150);
    }

    uint8_t rst = RSTCTRL.RSTFR;
    RSTCTRL.RSTFR = rst;
    usart_print("=== BOOT v2 hw-capture (RSTFR=0x");
    usart_putc("0123456789ABCDEF"[(rst >> 4) & 0xF]);
    usart_putc("0123456789ABCDEF"[rst & 0xF]);
    usart_print(") ===\r\n");

    uint16_t seq = 0;

    while (1) {
        sleep_until_button();
        seq++;

        opamp_on();
        adc_on();
        _delay_ms(2);                                /* op-amp settle */
        (void)adc_read(ADC_MUXPOS_AIN5_gc);          /* NTC warmup */

        PORTA.OUTSET = PIN4_bm;

        int16_t t_dC = ntc_read_temp_c16();

        uint16_t samples[MEDIAN_OF_N];
        uint8_t  valid = 0;
        for (uint8_t i = 0; i < MEDIAN_OF_N; i++) {
            uint16_t d = measure_distance_mm(t_dC);
            if (d != 0xFFFFu) samples[valid++] = d;
            _delay_ms(15);
        }

        PORTA.OUTCLR = PIN4_bm;
        adc_off();
        opamp_off();

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
            uint16_t d_mm = samples[valid / 2];   /* median of valid captures */
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
        usart_print(" hits, lastticks=");
        usart_print_u16(diag_ticks);
        usart_print(")\r\n");
    }
}
