/*
 * Bezkontaktowy miernik odleglosci - ATtiny1616, AVR-GCC (bez Arduino).
 * ToF ultradzwiekowy 40 kHz + docinanie faza (Goertzel) + kompensacja temp NTC.
 *
 * Piny (jak na plytce):
 *   PA0 UPDI
 *   PA3 przycisk (pullup, aktywny LOW)
 *   PA4 LED
 *   PA5 termistor NTC      -> ADC AIN5
 *   PA6 gate Q1/BSS84      LOW = wzmacniacz zasilony
 *   PB0 nadajnik 40 kHz    -> R1 -> BC817
 *   PB2 USART TX 115200
 *   PB4 echo z toru RX     -> ADC AIN9 (tez AC1 AINP3)
 *
 * 1 takt TCB0 = 100 ns (zegar 10 MHz). Odleglosc z onsetu obwiedni echa,
 * faze docina Goertzel. W spoczynku Standby + budzenie z RTC/PIT.
 */

#define F_CPU 10000000UL

#include <avr/io.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>

#define BAUD_RATE        115200UL
#define USART_BAUD_VAL   ((uint16_t)((64UL * F_CPU) / (16UL * BAUD_RATE)))

/* --- parametry pomiaru --- */
#define BURST_N          10         // krotki burst -> dziala juz od 10 cm
#define BURST_N_FAR      24         // dluzszy burst = mocniejsze echo na 50 cm
#define FAR_THRESH_MM    280        // powyzej tego przelacz na dluzszy burst
#define BLANKING_US      300        // przeczekanie ring-downu (~9.5 cm minimum)
#define ECHO_TIMEOUT_MS  4          // limit nasluchu (~65 cm)
#define NSAMP            320         // probek echa w buforze (~2.6 ms)
#define ECHO_MIN_DEV     6          // min. szczyt obwiedni zeby uznac echo (8-bit)
#define ONSET_DEV        5          // dolny prog onsetu (8-bit)
#define ONSET_FRAC       40u        // onset na tylu %% szczytu (niezalezny od amplitudy)
#define ADC_AIN_ECHO     ADC_MUXPOS_AIN9_gc   // PB4 = ADC AIN9
#define F0_HZ            40000.0f

/* --- faza (Goertzel) --- */
#define GOERTZEL_W       48         // szerokosc okna fazy (wokol szczytu)
#define PHASE_MIN_HITS   3          // min. pingow z faza zeby probowac fuzji
#define PLV_MIN          0.60f      // ponizej tego faza za zaszumiona -> bierz coarse
#define PHI0_RAD         0.0f       // offset fazy (opoznienie ukladu), strojony na wzorcu
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define DAC_THRESHOLD    255        // prog DAC ~2.49 V (tuz pod VG)
#define MEDIAN_OF_N      9          // tyle strzalow na pomiar, potem mediana
#define MIN_VALID_HITS   2          // tyle trafien minimum
#define MIN_TICKS        3000       // odrzuc ring-down (<~5 cm)
#define MAX_TICKS        40000      // odrzuc bzdury (>~68 cm)

/* --- kalibracja: real = a*raw/1000 + b. Domyslne z dopasowania 10-40 cm. --- */
#define CAL_A_NUM        989
#define CAL_A_DEN        1000
#define CAL_OFFSET_MM    (-72)

/* Kalibracja polowa (2 pkt) w EEPROM: a=(REF2-REF1)/(raw2-raw1), b=REF1-a*raw1 */
#define CAL_REF1_MM      100
#define CAL_REF2_MM      400
#define CAL_MAGIC        0x57E2u    // znacznik waznego rekordu w EEPROM

/* --- NTC 10k, model Beta. Dzielnik: +3V3 - NTC - PA5 - 10k - GND --- */
#define NTC_B            3950
#define NTC_R25          10000
#define NTC_R_PULLDOWN   10000
#define T_MIN_DC         (-200)
#define T_MAX_DC         (600)

/* ---- zegar ---- */
static void clock_init(void)
{
    // OSC20M / 2 = 10 MHz
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB,
                     CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm);
}

/* ---- piny ---- */
static void gpio_init(void)
{
    // przycisk z pullupem, czytany w petli
    PORTA.DIRCLR   = PIN3_bm;
    PORTA.PIN3CTRL = PORT_PULLUPEN_bm;

    PORTA.DIRSET = PIN4_bm; PORTA.OUTCLR = PIN4_bm;          // LED

    // PA5 wejscie analogowe NTC
    PORTA.DIRCLR   = PIN5_bm;
    PORTA.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;

    // gate Q1: HIGH = wzmacniacz odciety (spoczynek, oszczedza ~1 mA)
    PORTA.DIRSET = PIN6_bm; PORTA.OUTSET = PIN6_bm;

    PORTB.DIRSET = PIN0_bm; PORTB.OUTCLR = PIN0_bm;          // nadajnik
    PORTB.DIRSET = PIN2_bm; PORTB.OUTSET = PIN2_bm;          // USART TX

    // PB4 echo - wejscie analogowe
    PORTB.DIRCLR   = PIN4_bm;
    PORTB.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;

    // reszta pinow: wylacz bufory wejsc (mniej pradu, brak plywania)
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

static inline void opamp_on(void)  { PORTA.OUTCLR = PIN6_bm; }  // Q1 on
static inline void opamp_off(void) { PORTA.OUTSET = PIN6_bm; }  // Q1 off

/* ---- USART ---- */
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

// czekaj az ostatni bajt zejdzie z linii - inaczej Standby ucina go w polowie
// (TXCIF nie jest kasowany przy zapisie TXDATA, wiec prosciej odczekac jeden bajt)
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
static void usart_print_temp_dC(int16_t t)   // dziesiate stopnia -> "23.4"
{
    if (t < 0) { usart_putc('-'); t = (int16_t)-t; }
    usart_print_u16((uint16_t)(t / 10));
    usart_putc('.');
    usart_putc((char)('0' + t % 10));
}

/* ---- ADC ---- */
static void adc_init(void)
{
    ADC0.CTRLC    = ADC_PRESC_DIV8_gc | ADC_REFSEL_VDDREF_gc | ADC_SAMPCAP_bm;
    ADC0.CTRLD    = ADC_INITDLY_DLY16_gc;
    ADC0.SAMPCTRL = 0;
    ADC0.CTRLA    = ADC_RESSEL_10BIT_gc;          // wlaczany na zadanie
}
static inline void adc_on(void)  { ADC0.CTRLA |= ADC_ENABLE_bm; }
static inline void adc_off(void) { ADC0.CTRLA &= (uint8_t)~ADC_ENABLE_bm; }

// NTC: dokladne 10-bit, DIV8 (w spec), init delay na ustalenie referencji
static inline void adc_cfg_ntc(void)
{
    ADC0.CTRLC = ADC_PRESC_DIV8_gc | ADC_REFSEL_VDDREF_gc | ADC_SAMPCAP_bm;
    ADC0.CTRLD = ADC_INITDLY_DLY16_gc;
    ADC0.CTRLA = (ADC0.CTRLA & ~ADC_RESSEL_bm) | ADC_RESSEL_10BIT_gc;
}
// echo: szybkie 8-bit, DIV4 -> ~3 probki na okres 40 kHz
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
    adc_on();
    adc_cfg_ntc();
    // 8x oversampling -> mniej szumu temperatury
    uint32_t acc = 0;
    for (uint8_t k = 0; k < 8; k++) acc += adc_read(ADC_MUXPOS_AIN5_gc);
    uint16_t a = (uint16_t)(acc / 8);
    if (a == 0)    return T_MIN_DC;             // NTC w powietrzu / brak
    if (a >= 1023) return T_MAX_DC;

    float r     = (float)NTC_R_PULLDOWN * (float)(1023 - a) / (float)a;
    float inv_t = 1.0f / 298.15f + logf(r / (float)NTC_R25) / (float)NTC_B;
    float t_c   = 1.0f / inv_t - 273.15f;

    int32_t t_dc = (int32_t)(t_c * 10.0f + (t_c >= 0 ? 0.5f : -0.5f));
    if (t_dc < T_MIN_DC) t_dc = T_MIN_DC;
    if (t_dc > T_MAX_DC) t_dc = T_MAX_DC;
    return (int16_t)t_dc;
}

/* ---- AC1 + DAC1 (sprzetowa sciezka progowa, zweryfikowana) ----
 * Uwaga: na PB4 wejscie dodatnie to AINP3 komparatora AC1 (nie AC0!),
 * a AC1 korzysta z DAC1 - stad DAC1, nie DAC0. */
static void ac1_dac1_init(void)
{
    // referencja 2.5 V (musi byc <= VDD). DAC1REFSEL -> CTRLC, DAC1REFEN -> CTRLB
    VREF.CTRLC = (VREF.CTRLC & ~VREF_DAC1REFSEL_gm) | VREF_DAC1REFSEL_2V5_gc;
    VREF.CTRLB |= VREF_DAC1REFEN_bm;

    DAC1.DATA  = DAC_THRESHOLD;
    DAC1.CTRLA = DAC_ENABLE_bm;                    // tylko wewnetrznie

    AC1.MUXCTRLA = AC_MUXPOS_PIN3_gc | AC_MUXNEG_DAC_gc;   // +PB4, -DAC1
    AC1.CTRLA    = AC_ENABLE_bm | AC_HYSMODE_10mV_gc;
}

/* ---- EVSYS: AC1_OUT -> TCB0 ---- */
static void evsys_init(void)
{
    EVSYS.ASYNCCH0   = EVSYS_ASYNCCH0_AC1_OUT_gc;
    EVSYS.ASYNCUSER0 = EVSYS_ASYNCUSER0_ASYNCCH0_gc;
}

/* ---- TCB0: input capture + wolnobiezny licznik czasu ---- */
static void tcb0_init(void)
{
    TCB0.CTRLB    = TCB_CNTMODE_CAPT_gc;
    TCB0.EVCTRL   = TCB_CAPTEI_bm;
    TCB0.INTCTRL  = 0;
    TCB0.CNT      = 0;
    TCB0.INTFLAGS = TCB_CAPT_bm;
    TCB0.CTRLA    = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
}

/* ---- RTC/PIT: budzi CPU ze Standby co ~125 ms (na sprawdzenie przycisku) ---- */
static void rtc_pit_init(void)
{
    while (RTC.STATUS & RTC_CTRLBUSY_bm) { }
    RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;            // wewnetrzny ULP 32 kHz
    while (RTC.PITSTATUS & RTC_CTRLBUSY_bm) { }
    RTC.PITINTCTRL = RTC_PI_bm;
    RTC.PITCTRLA   = RTC_PERIOD_CYC4096_gc | RTC_PITEN_bm;  // 4096/32768 = 125 ms
}
ISR(RTC_PIT_vect) { RTC.PITINTFLAGS = RTC_PI_bm; }

/* ---- burst nadajnika ---- */
static void tx_burst(uint8_t n)
{
    // 12.5 us na polowke -> dokladnie 40.0 kHz (rezonans przetwornika)
    while (n--) {
        PORTB.OUTSET = PIN0_bm; _delay_us(12.5);
        PORTB.OUTCLR = PIN0_bm; _delay_us(12.5);
    }
}

/* ====== POMIAR ====== */
// surowe probki echa; onset/szczyt/Goertzel liczone juz po nabraniu bufora
static uint8_t  echo_raw[NSAMP];
static uint16_t echo_n;
static uint16_t echo_peak_idx;      // indeks szczytu obwiedni
static uint16_t echo_baseline;      // poziom spoczynkowy VG (8-bit)
static uint16_t echo_start_ticks;   // TCB0 przy pierwszej probce
static uint16_t echo_total_ticks;   // TCB0 na caly bufor
static uint16_t diag_peak;          // szczyt obwiedni (do diagnostyki)
static uint16_t diag_ticks;         // czas onsetu w taktach

// phasor fazy z jednego pingu (wektor jednostkowy ulamka cyklu)
static float   g_pcos, g_psin;
static uint8_t g_phase_ok;

/* Goertzel na buforze echa -> bezwzgledny ulamek cyklu.
 * Zapisuje phasor (cos,sin) zeby usrednic faze po pingach (szum ~1/sqrt(N)). */
static void compute_phase_phasor(void)
{
    g_phase_ok = 0;
    if (echo_total_ticks == 0 || diag_peak < ECHO_MIN_DEV) return;
    float ts_us = (float)echo_total_ticks / 10.0f / (float)NSAMP;
    if (ts_us < 1.0f) return;
    float omega = 2.0f * (float)M_PI * F0_HZ * ts_us * 1.0e-6f;
    if (omega < 0.1f || omega > (2.0f*(float)M_PI - 0.1f)) return;  // poza zakresem

    int32_t wstart = (int32_t)echo_peak_idx - GOERTZEL_W / 2;   // okno wokol szczytu
    if (wstart < 0) wstart = 0;
    if (wstart + GOERTZEL_W > (int32_t)NSAMP) wstart = (int32_t)NSAMP - GOERTZEL_W;
    if (wstart < 0) return;

    float cw = cosf(omega), sw = sinf(omega), coeff = 2.0f * cw, s1 = 0, s2 = 0;
    for (uint16_t k = 0; k < GOERTZEL_W; k++) {
        float x  = (float)((int16_t)echo_raw[wstart + k] - (int16_t)echo_baseline);
        float s0 = x + coeff * s1 - s2; s2 = s1; s1 = s0;
    }
    float re = s1 - s2 * cw, im = s2 * sw;
    float phase = atan2f(im, re);
    // czlon f0*t_okna przywraca info o odleglosci (okno jedzie ze szczytem)
    float period_ticks = (float)echo_total_ticks / (float)NSAMP;
    float t_ws_us = ((float)echo_start_ticks + (float)wstart * period_ticks) / 10.0f;
    float cycles  = F0_HZ * t_ws_us * 1.0e-6f
                  - phase / (2.0f * (float)M_PI)
                  + PHI0_RAD / (2.0f * (float)M_PI);
    float frac = cycles - floorf(cycles);
    float ang  = 2.0f * (float)M_PI * frac;
    g_pcos = cosf(ang); g_psin = sinf(ang); g_phase_ok = 1;
}

/* Jeden ping. Minimalna petla nabiera bufor echa; zwraca czas onsetu w taktach
 * TCB0 (0 = brak echa) i liczy phasor fazy dla tego pingu. */
static uint16_t measure_once_ticks(uint8_t n_cycles)
{
    adc_cfg_echo();
    echo_baseline = adc_read(ADC_AIN_ECHO);

    PORTB.OUTCLR = PIN0_bm;
    TCB0.CNT     = 0;                            // zero = start burstu
    tx_burst(n_cycles);
    PORTB.OUTCLR = PIN0_bm;
    _delay_us(BLANKING_US);

    ADC0.MUXPOS = ADC_AIN_ECHO;
    uint16_t start = TCB0.CNT;
    for (uint16_t i = 0; i < NSAMP; i++) {       // ciasna, rownomierna petla
        ADC0.INTFLAGS = ADC_RESRDY_bm;
        ADC0.COMMAND  = ADC_STCONV_bm;
        while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) { }
        echo_raw[i] = (uint8_t)ADC0.RES;
    }
    uint16_t end = TCB0.CNT;
    echo_n = NSAMP; echo_start_ticks = start;
    echo_total_ticks = (uint16_t)(end - start);
    g_phase_ok = 0;

    // krok 1: szczyt obwiedni
    uint16_t peak_dev = 0, peak_idx = 0;
    for (uint16_t i = 0; i < NSAMP; i++) {
        int16_t dev  = (int16_t)echo_raw[i] - (int16_t)echo_baseline;
        int16_t adev = dev < 0 ? -dev : dev;
        if (adev > (int16_t)peak_dev) { peak_dev = (uint16_t)adev; peak_idx = i; }
    }
    echo_peak_idx = peak_idx;
    diag_peak     = peak_dev;
    if (peak_dev < ECHO_MIN_DEV) { diag_ticks = 0; return 0; }

    // krok 2: onset = prog 40% szczytu (niezalezny od amplitudy), z interpolacja
    int16_t thr = (int16_t)(((uint32_t)peak_dev * ONSET_FRAC) / 100u);
    if (thr < ONSET_DEV) thr = ONSET_DEV;
    uint8_t onset_found = 0; float onset_idx_f = 0.0f; int16_t prev_adev = 0;
    for (uint16_t i = 0; i <= peak_idx; i++) {
        int16_t dev  = (int16_t)echo_raw[i] - (int16_t)echo_baseline;
        int16_t adev = dev < 0 ? -dev : dev;
        if (adev >= thr) {
            float fr = 0.0f;     // interpolacja przejscia progu miedzy probkami
            if (i > 0 && adev > prev_adev)
                fr = (float)(thr - prev_adev) / (float)(adev - prev_adev);
            onset_idx_f = (float)(int16_t)(i - 1) + fr;
            if (onset_idx_f < 0.0f) onset_idx_f = 0.0f;
            onset_found = 1; break;
        }
        prev_adev = adev;
    }
    if (!onset_found) { diag_ticks = 0; return 0; }

    float period = (float)echo_total_ticks / (float)NSAMP;
    uint32_t onset_ticks = (uint32_t)start + (uint32_t)(onset_idx_f * period + 0.5f);
    diag_ticks = (uint16_t)onset_ticks;
    if (onset_ticks < MIN_TICKS || onset_ticks > MAX_TICKS) return 0;  // bramka

    compute_phase_phasor();
    return (uint16_t)onset_ticks;
}

// aktywna kalibracja (z EEPROM jak wazna, inaczej domyslne)
static int16_t g_cal_a = CAL_A_NUM;     // nachylenie x1000
static int16_t g_cal_b = CAL_OFFSET_MM; // offset [mm]

// surowa odleglosc z taktow (czysta geometria ToF)
static int32_t ticks_to_raw_mm(int32_t c_x1000, uint32_t ticks)
{
    return (int32_t)(((int64_t)c_x1000 * (int64_t)ticks) / 20000000LL);
}

// odleglosc po kalibracji: real = a*raw + b
static uint16_t ticks_to_mm(int32_t c_x1000, uint32_t ticks)
{
    int32_t raw = ticks_to_raw_mm(c_x1000, ticks);
    int32_t d   = (raw * (int32_t)g_cal_a) / 1000 + (int32_t)g_cal_b;
    if (d < 0)      d = 0;
    if (d > 0xFFFE) d = 0xFFFE;
    return (uint16_t)d;
}

/* ====== KALIBRACJA W EEPROM ====== */
typedef struct { uint16_t magic; int16_t a_x1000; int16_t b_mm; uint8_t crc; } cal_t;
static cal_t EEMEM ee_cal;

static uint8_t cal_crc8(const cal_t *c)
{
    const uint8_t *p = (const uint8_t *)c;
    uint8_t crc = 0;
    for (uint8_t i = 0; i < offsetof(cal_t, crc); i++) {
        crc ^= p[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

// wczytaj rekord; jak zly (magic/CRC/zakres) zostaw domyslne
static void cal_load(void)
{
    cal_t c;
    eeprom_read_block(&c, &ee_cal, sizeof(c));
    if (c.magic == CAL_MAGIC && c.crc == cal_crc8(&c) &&
        c.a_x1000 > 500 && c.a_x1000 < 1500) {
        g_cal_a = c.a_x1000;
        g_cal_b = c.b_mm;
    }
}

static void cal_save(int16_t a_x1000, int16_t b_mm)
{
    cal_t c = { CAL_MAGIC, a_x1000, b_mm, 0 };
    c.crc = cal_crc8(&c);
    eeprom_update_block(&c, &ee_cal, sizeof(c));
    g_cal_a = a_x1000; g_cal_b = b_mm;
}

/* ====== FUZJA FAZY ======
 * Laczy odleglosc zgrubna z usredniona faza serii (phasory sumc/sums, npz pingow).
 * Faza dociaga do siatki lambda/2; jak PLV niskie albo rozjazd > lambda/4 -> coarse. */
static uint16_t phase_fuse_mm(int32_t c_x1000, uint16_t d_coarse,
                              float sumc, float sums, uint8_t npz,
                              int16_t *phase_d10, uint8_t *plv_pct)
{
    *phase_d10 = -3600; *plv_pct = 0;
    if (npz < PHASE_MIN_HITS) return d_coarse;

    float mag = sqrtf(sumc*sumc + sums*sums);
    float plv = mag / (float)npz;                 // phase-lock value (spojnosc fazy)
    *plv_pct  = (uint8_t)(plv * 100.0f + 0.5f);

    float frac = atan2f(sums, sumc) / (2.0f * (float)M_PI);
    if (frac < 0.0f) frac += 1.0f;
    *phase_d10 = (int16_t)lroundf(frac * 3600.0f);

    if (plv < PLV_MIN) return d_coarse;           // faza nie zlapana

    float lam_half = (float)c_x1000 / (2.0f * F0_HZ);
    float phase_mm = frac * lam_half;
    float nf       = roundf(((float)d_coarse - phase_mm) / lam_half);  // numer prazka
    float dfine    = nf * lam_half + phase_mm;
    if (fabsf(dfine - (float)d_coarse) > lam_half * 0.5f) return d_coarse;
    if (dfine < 0.0f) dfine = 0.0f;
    if (dfine > 65534.0f) dfine = 65534.0f;
    return (uint16_t)lroundf(dfine);
}

/* ====== SERIA POMIAROWA ====== */
static float    g_sumc, g_sums;     // suma phasorow fazy
static uint8_t  g_npz, g_nv;        // pingi z faza / wszystkie trafienia

/* Seria z adaptacyjnym burstem i odpornym usrednianiem (mediana + MAD) na taktach.
 * Zwraca onset w taktach (0 = brak), wypelnia phasory i licznik trafien.
 * Sama wlacza/wylacza tor analogowy. */
static uint32_t measure_coarse_ticks(int32_t c_x1000)
{
    opamp_on(); adc_on(); _delay_ms(5);          // rozruch wzmacniacza
    (void)adc_read(ADC_MUXPOS_AIN5_gc);

    // ping wstepny -> wybor dlugosci burstu
    uint16_t ct    = measure_once_ticks(BURST_N);
    uint8_t  n_use = (ct == 0 || ticks_to_mm(c_x1000, ct) > FAR_THRESH_MM)
                     ? BURST_N_FAR : BURST_N;

    uint16_t tk[MEDIAN_OF_N];
    uint8_t  nv = 0, npz = 0;
    float    sumc = 0.0f, sums = 0.0f;
    for (uint8_t i = 0; i < MEDIAN_OF_N; i++) {
        uint16_t t = measure_once_ticks(n_use);
        if (t) {
            tk[nv++] = t;
            if (g_phase_ok) { sumc += g_pcos; sums += g_psin; npz++; }
        }
        _delay_ms(15);
    }
    adc_off(); opamp_off();

    g_nv = nv; g_npz = npz; g_sumc = sumc; g_sums = sums;
    if (nv < MIN_VALID_HITS) return 0;

    // sortuj -> mediana
    for (uint8_t i = 1; i < nv; i++) {
        uint16_t v = tk[i]; int8_t j = i - 1;
        while (j >= 0 && tk[j] > v) { tk[j+1] = tk[j]; j--; } tk[j+1] = v;
    }
    uint16_t med = tk[nv / 2];
    // MAD: mediana odchylen -> srednia z probek mieszczacych sie w 3*MAD
    uint16_t dv[MEDIAN_OF_N];
    for (uint8_t i = 0; i < nv; i++) dv[i] = tk[i] > med ? tk[i]-med : med-tk[i];
    for (uint8_t i = 1; i < nv; i++) {
        uint16_t v = dv[i]; int8_t j = i - 1;
        while (j >= 0 && dv[j] > v) { dv[j+1] = dv[j]; j--; } dv[j+1] = v;
    }
    uint16_t lim = (uint16_t)(dv[nv/2] * 3u) + 2u;
    uint32_t sum = 0; uint8_t cnt = 0;
    for (uint8_t i = 0; i < nv; i++) {
        uint16_t d = tk[i] > med ? tk[i]-med : med-tk[i];
        if (d <= lim) { sum += tk[i]; cnt++; }
    }
    return cnt ? (sum / cnt) : med;
}

/* ---- przycisk / uspienie ---- */
static uint8_t button_pressed(void) { return (PORTA.IN & PIN3_bm) == 0; }

// spij w Standby; PIT budzi co ~125 ms, sprawdzamy przycisk
static void sleep_until_button(void)
{
    for (;;) {
        set_sleep_mode(SLEEP_MODE_STANDBY);
        sleep_enable();
        sei();
        sleep_cpu();
        sleep_disable();

        if (button_pressed()) {
            _delay_ms(20);           // debounce
            if (button_pressed()) return;
        }
    }
}

/* ====== TRYB KALIBRACJI ====== */
// czekaj (aktywnie) na klik z debounce
static void wait_press_release(void)
{
    while (button_pressed()) _delay_ms(5);          // najpierw puszczenie
    _delay_ms(20);
    while (!button_pressed()) _delay_ms(5);         // nacisniecie
    _delay_ms(20);
    while (button_pressed()) _delay_ms(5);          // puszczenie
    _delay_ms(20);
}

static void cal_blink(uint8_t n)
{
    while (n--) { PORTA.OUTSET = PIN4_bm; _delay_ms(80);
                  PORTA.OUTCLR = PIN4_bm; _delay_ms(120); }
}

/* Kalibracja 2-punktowa - wejscie: trzymaj przycisk przy starcie.
 * Prowadzi (USART + LED) przez ustawienie celu na 100 i 400 mm, liczy a,b
 * i zapisuje do EEPROM. */
static void cal_mode(void)
{
    int16_t t_dC    = ntc_read_temp_c16();
    int32_t c_x1000 = 331300L + ((int32_t)606 * (int32_t)t_dC) / 10L;

    usart_print("\r\n=== KALIBRACJA ===\r\n");
    usart_print("Ustaw cel na 100 mm, klik...\r\n"); usart_flush();
    cal_blink(1);
    wait_press_release();
    uint32_t tk1 = measure_coarse_ticks(c_x1000);
    if (!tk1) { usart_print("REF1 brak echa - przerwano\r\n"); usart_flush(); return; }
    int32_t raw1 = ticks_to_raw_mm(c_x1000, tk1);

    usart_print("Ustaw cel na 400 mm, klik...\r\n"); usart_flush();
    cal_blink(2);
    wait_press_release();
    uint32_t tk2 = measure_coarse_ticks(c_x1000);
    if (!tk2) { usart_print("REF2 brak echa - przerwano\r\n"); usart_flush(); return; }
    int32_t raw2 = ticks_to_raw_mm(c_x1000, tk2);

    if (raw2 <= raw1 + 10) { usart_print("Bledne punkty - przerwano\r\n"); usart_flush(); return; }

    float a = (float)(CAL_REF2_MM - CAL_REF1_MM) / (float)(raw2 - raw1);
    float b = (float)CAL_REF1_MM - a * (float)raw1;
    int16_t a_x1000 = (int16_t)lroundf(a * 1000.0f);
    int16_t b_mm    = (int16_t)lroundf(b);
    cal_save(a_x1000, b_mm);

    usart_print("Zapisano EEPROM: a=");
    usart_print_u16((uint16_t)a_x1000);
    usart_print(" b=");
    if (b_mm < 0) { usart_putc('-'); b_mm = (int16_t)-b_mm; }
    usart_print_u16((uint16_t)b_mm);
    usart_print("\r\n"); usart_flush();
    cal_blink(5);
}

/* ====== MAIN ====== */
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

    cal_load();                                       // EEPROM albo domyslne

    uint8_t rst = RSTCTRL.RSTFR;
    RSTCTRL.RSTFR = rst;
    usart_print("=== WETI ultrasonic meter (Etap2) RSTFR=0x");
    usart_putc("0123456789ABCDEF"[(rst >> 4) & 0xF]);
    usart_putc("0123456789ABCDEF"[rst & 0xF]);
    usart_print(g_cal_a != CAL_A_NUM || g_cal_b != CAL_OFFSET_MM
                ? " cal=EEPROM ===\r\n" : " cal=default ===\r\n");
    usart_flush();

    // trzymanie przycisku przy starcie = wejscie w kalibracje (sprawdzane wczesnie)
    if (button_pressed()) {
        _delay_ms(50);
        if (button_pressed()) cal_mode();
    }

    // mrugniecie startowe = gotowy
    for (uint8_t i = 0; i < 3; i++) {
        PORTA.OUTSET = PIN4_bm; _delay_ms(150);
        PORTA.OUTCLR = PIN4_bm; _delay_ms(150);
    }

    uint16_t seq = 0;

    for (;;) {
        sleep_until_button();
        seq++;
        PORTA.OUTSET = PIN4_bm;                        // LED na czas pomiaru

        int16_t  t_dC    = ntc_read_temp_c16();
        int32_t  c_x1000 = 331300L + ((int32_t)606 * (int32_t)t_dC) / 10L;
        uint32_t coarse_ticks = measure_coarse_ticks(c_x1000);

        PORTA.OUTCLR = PIN4_bm;

        usart_print("#");
        usart_print_u16(seq);
        usart_putc(' ');
        if (g_nv < MIN_VALID_HITS || coarse_ticks == 0) {
            usart_print("NO ECHO T=");
            usart_print_temp_dC(t_dC);
            usart_print("C (");
            usart_print_u16(g_nv);
            usart_print("/");
            usart_print_u16(MEDIAN_OF_N);
            usart_print(")");
        } else {
            uint16_t d_coarse = ticks_to_mm(c_x1000, coarse_ticks);
            int16_t  ph_d10; uint8_t plv_pct;
            uint16_t d_fine = phase_fuse_mm(c_x1000, d_coarse,
                                            g_sumc, g_sums, g_npz, &ph_d10, &plv_pct);

            usart_print("D=");
            usart_print_u16(d_fine);
            usart_print("mm (Dc=");
            usart_print_u16(d_coarse);
            usart_print(" ph=");
            if (ph_d10 == -3600) {
                usart_print("--");
            } else {
                usart_print_u16((uint16_t)(ph_d10 / 10));
                usart_putc('.');
                usart_putc((char)('0' + ph_d10 % 10));
            }
            usart_print(" plv=");
            usart_print_u16(plv_pct);
            usart_print(" T=");
            usart_print_temp_dC(t_dC);
            usart_print("C ");
            usart_print_u16(g_nv);
            usart_print("/");
            usart_print_u16(MEDIAN_OF_N);
            usart_print(" pk=");
            usart_print_u16(diag_peak);
            usart_print(")");
        }
        usart_print("\r\n");
        usart_flush();

        while (button_pressed()) _delay_ms(10);       // 1 klik = 1 pomiar
    }
}
