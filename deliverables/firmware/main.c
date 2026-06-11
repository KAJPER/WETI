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
#define BURST_N          10         /* fixed burst (250us) */
#define BLANKING_US      300        /* ring-down skip; ~9.5 cm floor */
#define ECHO_TIMEOUT_MS  4          /* covers ~65 cm */
#define NSAMP            320        /* echo samples (~2.6 ms window @ ~8 us) */
#define ECHO_MIN_DEV     8          /* min envelope peak to accept echo (8-bit) */
#define ONSET_DEV        7          /* first-crossing threshold = echo arrival (8-bit) */
#define ADC_AIN_ECHO     ADC_MUXPOS_AIN9_gc   /* PB4 = ADC0 AIN9 */
#define F0_HZ            40000.0f   /* burst / echo carrier frequency */

/* ----- Goertzel phase stage ----- */
#define GOERTZEL_W       48         /* samples in the phase window (centered on peak) */
/* Phase calibration offset [rad]: system/electrical delay. Tune so that a
 * known distance reads correctly (see procedure in docs). Default 0. */
#define PHI0_RAD         0.0f
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
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
#define CAL_OFFSET_MM    (-70)      /* envelope-rise delay (~400us). Verify @2 dist. */

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

/* Wait until the last byte has fully shifted out, then it is safe to sleep.
 * DREIF set => last byte moved from DATA to the shift register; one byte time
 * (~87us @115200) later it is fully on the wire. Delay 150us to be safe. This
 * avoids the TXCIF race (TXCIF is NOT auto-cleared on TXDATA write). */
static void usart_flush(void)
{
    while (!(USART0.STATUS & USART_DREIF_bm)) { }
    _delay_us(150);
}
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

/* ============================== ADC ================================= */
static void adc_init(void)
{
    ADC0.CTRLC    = ADC_PRESC_DIV8_gc | ADC_REFSEL_VDDREF_gc | ADC_SAMPCAP_bm;
    ADC0.CTRLD    = ADC_INITDLY_DLY16_gc;
    ADC0.SAMPCTRL = 0;
    ADC0.CTRLA    = ADC_RESSEL_10BIT_gc;          /* enabled on demand */
}
static inline void adc_on(void)  { ADC0.CTRLA |= ADC_ENABLE_bm; }
static inline void adc_off(void) { ADC0.CTRLA &= (uint8_t)~ADC_ENABLE_bm; }

/* NTC: accurate 10-bit, DIV8 (in spec), init delay for clean reference. */
static inline void adc_cfg_ntc(void)
{
    ADC0.CTRLC = ADC_PRESC_DIV8_gc | ADC_REFSEL_VDDREF_gc | ADC_SAMPCAP_bm;
    ADC0.CTRLD = ADC_INITDLY_DLY16_gc;
    ADC0.CTRLA = (ADC0.CTRLA & ~ADC_RESSEL_bm) | ADC_RESSEL_10BIT_gc;
}
/* Echo: fast 8-bit, DIV4, no init delay -> ~3 samples per 40 kHz cycle. */
static inline void adc_cfg_echo(void)
{
    ADC0.CTRLC = ADC_PRESC_DIV4_gc | ADC_REFSEL_VDDREF_gc | ADC_SAMPCAP_bm;
    ADC0.CTRLD = 0;
    ADC0.CTRLA = (ADC0.CTRLA & ~ADC_RESSEL_bm) | ADC_RESSEL_8BIT_gc;
}

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
    adc_cfg_ntc();                                /* NTC needs full resolution */
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
    /* 12.5 us half-period -> exactly 40.0 kHz (on transducer resonance) */
    while (n--) {
        PORTB.OUTSET = PIN0_bm; _delay_us(12.5);
        PORTB.OUTCLR = PIN0_bm; _delay_us(12.5);
    }
}

/* ============================== DISTANCE ============================ */
/* Raw 8-bit echo samples (filled by a minimal tight loop for max sample rate;
 * onset/peak/Goertzel run afterwards in post-processing). */
static uint8_t  echo_raw[NSAMP];
static uint16_t echo_n;             /* = NSAMP */
static uint16_t echo_peak_idx;      /* index of envelope peak */
static uint16_t echo_baseline;      /* ADC baseline (VG), 8-bit */
static uint16_t echo_start_ticks;   /* TCB0 ticks at first sample */
static uint16_t echo_total_ticks;   /* TCB0 ticks spanning NSAMP samples */
static uint16_t diag_peak;          /* peak |deviation| (8-bit counts) */
static uint16_t diag_ticks;         /* TCB0 ticks at echo onset */

/* One ping. A MINIMAL loop ADC-samples the echo on PB4 (AIN9) as fast as
 * possible (uniform rate). Onset (echo arrival), envelope peak, and the
 * Goertzel window are derived afterwards. The sample period is recovered
 * from the TCB0 span (start..end), giving an accurate time-of-flight.
 * Returns calibrated mm, or 0xFFFF if no valid echo. */
static uint16_t measure_once_mm(int16_t t_c_dC, uint8_t n_cycles)
{
    int32_t c_x1000 = 331300L + ((int32_t)606 * (int32_t)t_c_dC) / 10L;

    adc_cfg_echo();
    echo_baseline = adc_read(ADC_AIN_ECHO);      /* VG baseline, 8-bit (~208) */

    PORTB.OUTCLR = PIN0_bm;
    TCB0.CNT     = 0;                            /* time reference = burst start */
    tx_burst(n_cycles);
    PORTB.OUTCLR = PIN0_bm;

    _delay_us(BLANKING_US);                      /* skip ring-down */

    ADC0.MUXPOS = ADC_AIN_ECHO;
    uint16_t start = TCB0.CNT;
    for (uint16_t i = 0; i < NSAMP; i++) {       /* tight uniform sampling */
        ADC0.INTFLAGS = ADC_RESRDY_bm;
        ADC0.COMMAND  = ADC_STCONV_bm;
        while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) { }
        echo_raw[i] = (uint8_t)ADC0.RES;
    }
    uint16_t end = TCB0.CNT;

    echo_n           = NSAMP;
    echo_start_ticks = start;
    echo_total_ticks = (uint16_t)(end - start);

    /* post-process: onset (first crossing) + envelope peak */
    uint16_t peak_dev = 0, peak_idx = 0, onset_idx = 0;
    uint8_t  onset_found = 0;
    for (uint16_t i = 0; i < NSAMP; i++) {
        int16_t dev  = (int16_t)echo_raw[i] - (int16_t)echo_baseline;
        int16_t adev = dev < 0 ? -dev : dev;
        if (!onset_found && adev >= ONSET_DEV) { onset_idx = i; onset_found = 1; }
        if (adev > (int16_t)peak_dev) { peak_dev = (uint16_t)adev; peak_idx = i; }
    }
    echo_peak_idx = peak_idx;
    diag_peak     = peak_dev;

    if (!onset_found || peak_dev < ECHO_MIN_DEV) { diag_ticks = 0; return 0xFFFFu; }

    /* absolute onset time = start + onset_idx * (period); period = span/NSAMP */
    uint32_t onset_ticks = (uint32_t)start +
        ((uint32_t)onset_idx * (uint32_t)echo_total_ticks) / NSAMP;
    diag_ticks = (uint16_t)onset_ticks;

    int32_t raw = (int32_t)(((int64_t)c_x1000 * (int64_t)onset_ticks) / 20000000LL);
    int32_t d   = (raw * CAL_A_NUM) / CAL_A_DEN + CAL_OFFSET_MM;
    if (d < 0)      d = 0;
    if (d > 0xFFFE) d = 0xFFFE;
    return (uint16_t)d;
}

/* ============================== GOERTZEL / PHASE =================== */
/* Pulse-Phase fusion: refine the coarse ToF distance using the carrier phase
 * of the echo. Goertzel runs at F0 over a window centered on the envelope
 * peak; the phase pins the distance to a lambda/2 grid (4.3 mm), refining the
 * coarse estimate to sub-millimetre IF coarse error < lambda/4. A consistency
 * check falls back to the coarse value when the phase disagrees.
 *   d_coarse : robust coarse distance [mm] (median of ToF pings)
 *   phase_d10: output, echo phase in deci-degrees (or -3600 if unavailable)
 * Returns the fused distance [mm]. */
static uint16_t phase_refine_mm(int32_t c_x1000, uint16_t d_coarse,
                                int16_t *phase_d10)
{
    *phase_d10 = -3600;
    if (echo_n < GOERTZEL_W || echo_total_ticks == 0 ||
        diag_peak < ECHO_MIN_DEV)
        return d_coarse;

    /* sample rate from measured total sampling time */
    float ts_us = (float)echo_total_ticks / 10.0f / (float)echo_n;  /* us/sample */
    if (ts_us < 1.0f) return d_coarse;
    float fs    = 1.0e6f / ts_us;
    float omega = 2.0f * (float)M_PI * F0_HZ / fs;
    if (omega < 0.1f || omega > (2.0f*(float)M_PI - 0.1f)) return d_coarse;

    /* window centered on the envelope peak, clamped to the buffer */
    int32_t start = (int32_t)echo_peak_idx - GOERTZEL_W / 2;
    if (start < 0) start = 0;
    if (start + GOERTZEL_W > (int32_t)echo_n) start = (int32_t)echo_n - GOERTZEL_W;
    if (start < 0) return d_coarse;

    float cw = cosf(omega), sw = sinf(omega), coeff = 2.0f * cw;
    float s1 = 0.0f, s2 = 0.0f;
    for (uint16_t k = 0; k < GOERTZEL_W; k++) {
        float x  = (float)((int16_t)echo_raw[start + k] - (int16_t)echo_baseline);
        float s0 = x + coeff * s1 - s2;
        s2 = s1; s1 = s0;
    }
    float re = s1 - s2 * cw;
    float im = s2 * sw;
    float phase = atan2f(im, re);                       /* carrier phase @ t_ws */

    /* Absolute carrier cycles from burst start to the window-start time encode
     * the round-trip distance: echo(t) = cos(2pi f0 (t - ToF) + phi0), so the
     * Goertzel phase at t_ws = 2pi f0 (t_ws - ToF) + phi0. Hence
     *   distance/(lambda/2) = f0*ToF = f0*t_ws + (phi0 - phase)/(2pi).
     * Including f0*t_ws restores the distance information lost by centering the
     * window on the (moving) envelope peak. */
    float period_ticks = (float)echo_total_ticks / (float)NSAMP;
    float t_ws_us = ((float)echo_start_ticks + (float)start * period_ticks) / 10.0f;
    float cycles  = F0_HZ * t_ws_us * 1.0e-6f
                  - phase / (2.0f * (float)M_PI)
                  + PHI0_RAD / (2.0f * (float)M_PI);
    float frac    = cycles - floorf(cycles);            /* [0,1) cycle */
    *phase_d10 = (int16_t)lroundf(frac * 3600.0f);      /* 0..3600 deci-deg */

    /* lambda/2 [mm] = c / (2 f0); c_x1000 = c[m/s]*1000 */
    float lam_half = (float)c_x1000 / (2.0f * F0_HZ);
    float phase_mm = frac * lam_half;

    float nf    = roundf(((float)d_coarse - phase_mm) / lam_half);
    float dfine = nf * lam_half + phase_mm;

    /* consistency: reject lambda/2 bin jumps */
    if (fabsf(dfine - (float)d_coarse) > lam_half * 0.5f)
        dfine = (float)d_coarse;

    if (dfine < 0.0f)       dfine = 0.0f;
    if (dfine > 65534.0f)   dfine = 65534.0f;
    return (uint16_t)lroundf(dfine);
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
    usart_flush();

    uint16_t seq = 0;

    for (;;) {
        sleep_until_button();
        seq++;

        /* Power up analog front-end for the measurement */
        opamp_on();
        adc_on();
        _delay_ms(5);                                /* op-amp / VDD_AMP settle */
        (void)adc_read(ADC_MUXPOS_AIN5_gc);          /* ADC warm-up */

        PORTA.OUTSET = PIN4_bm;                       /* LED on */

        int16_t t_dC = ntc_read_temp_c16();

        uint16_t samples[MEDIAN_OF_N];
        uint8_t  valid = 0;
        for (uint8_t i = 0; i < MEDIAN_OF_N; i++) {
            uint16_t d = measure_once_mm(t_dC, BURST_N);
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
            usart_print(" T=");
            usart_print_temp_dC(t_dC);
            usart_print("C (");
            usart_print_u16(valid);
            usart_print("/");
            usart_print_u16(MEDIAN_OF_N);
            usart_print(")");
        } else {
            uint16_t d_coarse = samples[valid / 2];
            int32_t  c_x1000  = 331300L + ((int32_t)606 * (int32_t)t_dC) / 10L;
            int16_t  ph_d10;
            uint16_t d_fine   = phase_refine_mm(c_x1000, d_coarse, &ph_d10);

            usart_print("D=");
            usart_print_u16(d_fine);
            usart_print("mm (Dc=");
            usart_print_u16(d_coarse);
            usart_print(" ph=");
            if (ph_d10 == -3600) {
                usart_print("--");
            } else {
                if (ph_d10 < 0) { usart_putc('-'); ph_d10 = (int16_t)-ph_d10; }
                usart_print_u16((uint16_t)(ph_d10 / 10));
                usart_putc('.');
                usart_putc((char)('0' + ph_d10 % 10));
            }
            usart_print(" T=");
            usart_print_temp_dC(t_dC);
            usart_print("C ");
            usart_print_u16(valid);
            usart_print("/");
            usart_print_u16(MEDIAN_OF_N);
            usart_print(" pk=");
            usart_print_u16(diag_peak);
            usart_print(")");
        }
        usart_print("\r\n");
        usart_flush();                    /* drain TX before sleeping (no garble) */

        /* wait for release so one press = one measurement */
        while (button_pressed()) _delay_ms(10);
    }
}
