// ============================================================
// pico_feedback.c — Joint position feedback for shoulder + elbow
//
// Reads two 270° potentiometers via ADC and reports actual joint
// angles over USB serial at 50 Hz.
//
// Also accepts commanded angles over serial and flags drift.
// Designed to slot into micro-ROS later — replace printf/getchar
// with topic publishes/subscribes, keep the logic unchanged.
//
// Hardware:
//   GP26 (ADC0) — shoulder pot wiper
//   GP27 (ADC1) — elbow pot wiper
//   Pot ends wired to 3.3V and AGND (use AGND pin, not GND)
//   Add 100nF cap between wiper and AGND at each pot
//
// Serial protocol (115200 baud, USB):
//   Output:  POS shoulder:<deg> elbow:<deg>
//   Output:  DRIFT shoulder:<signed_deg> elbow:<signed_deg>
//   Input:   CMD shoulder:<deg> elbow:<deg>
//   Input:   STATUS
//   Input:   HELP
// ============================================================

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

// ============================================================
// Config — edit to match hardware
// ============================================================
#define ADC_CH_SHOULDER     0           // GP26 = ADC0
#define ADC_CH_ELBOW        1           // GP27 = ADC1
#define ADC_SAMPLES         16          // readings averaged per sample
#define POT_RANGE_DEG       270.0f      // mechanical rotation of pot
#define DRIFT_THRESHOLD_DEG 2.0f        // degrees before DRIFT line prints
#define REPORT_HZ           50          // position report rate
#define REPORT_INTERVAL_MS  (1000 / REPORT_HZ)
#define SERIAL_BUF_LEN      80

// ============================================================
// State
// ============================================================
static float cmd_shoulder  = 0.0f;
static float cmd_elbow     = 0.0f;
static bool  cmd_active    = false;     // true once first CMD received

// ============================================================
// ADC
// ============================================================

// Read one ADC channel, average ADC_SAMPLES readings.
// 12-bit result (0–4095) mapped to 0–POT_RANGE_DEG.
static float read_pot_deg(uint channel) {
    adc_select_input(channel);
    uint32_t sum = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        sum += adc_read();
    }
    float avg = (float)sum / (float)ADC_SAMPLES;
    return (avg / 4095.0f) * POT_RANGE_DEG;
}

// ============================================================
// Serial command parser
// ============================================================

static void print_help(void) {
    printf("=== PICO FEEDBACK COMMANDS ===\n");
    printf("CMD shoulder:<deg> elbow:<deg>  — set commanded angles for drift check\n");
    printf("STATUS                          — print current config\n");
    printf("HELP                            — this list\n");
    printf("==============================\n");
}

static void print_status(float shoulder, float elbow) {
    printf("=== STATUS ===\n");
    printf("Shoulder actual : %.2f deg\n", shoulder);
    printf("Elbow actual    : %.2f deg\n", elbow);
    printf("Shoulder cmd    : %.2f deg\n", cmd_shoulder);
    printf("Elbow cmd       : %.2f deg\n", cmd_elbow);
    printf("Drift threshold : %.2f deg\n", DRIFT_THRESHOLD_DEG);
    printf("Report rate     : %d Hz\n",    REPORT_HZ);
    printf("ADC samples     : %d\n",       ADC_SAMPLES);
    printf("Drift active    : %s\n",       cmd_active ? "yes" : "no (no CMD received yet)");
    printf("==============\n");
}

// Parse one null-terminated line from serial.
// Returns true if STATUS was requested (needs fresh ADC reading).
static bool parse_line(const char *line, bool *want_status) {
    *want_status = false;

    // CMD shoulder:<deg> elbow:<deg>
    float s, e;
    if (sscanf(line, "CMD shoulder:%f elbow:%f", &s, &e) == 2) {
        cmd_shoulder = s;
        cmd_elbow    = e;
        cmd_active   = true;
        printf("CMD ACK shoulder:%.2f elbow:%.2f\n", s, e);
        return true;
    }

    if (strncmp(line, "STATUS", 6) == 0) {
        *want_status = true;
        return true;
    }

    if (strncmp(line, "HELP", 4) == 0) {
        print_help();
        return true;
    }

    printf("ERR unknown command: %s\n", line);
    return false;
}

// ============================================================
// Main
// ============================================================
int main(void) {
    stdio_init_all();

    // ADC init
    adc_init();
    adc_gpio_init(26);  // shoulder — GP26 = ADC0
    adc_gpio_init(27);  // elbow   — GP27 = ADC1

    // Wait for USB serial to connect (PC-side terminal)
    sleep_ms(2000);
    printf("=== Pico Feedback Ready ===\n");
    printf("Joints: shoulder (GP26), elbow (GP27)\n");
    printf("Type HELP for commands.\n");

    char   buf[SERIAL_BUF_LEN];
    int    buf_pos   = 0;
    uint32_t last_report_ms = to_ms_since_boot(get_absolute_time());

    while (true) {

        // --- Non-blocking serial read ---
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            if (c == '\n' || c == '\r') {
                if (buf_pos > 0) {
                    buf[buf_pos] = '\0';
                    buf_pos = 0;
                    bool want_status = false;
                    parse_line(buf, &want_status);
                    if (want_status) {
                        float sh = read_pot_deg(ADC_CH_SHOULDER);
                        float el = read_pot_deg(ADC_CH_ELBOW);
                        print_status(sh, el);
                    }
                }
            } else if (buf_pos < SERIAL_BUF_LEN - 1) {
                buf[buf_pos++] = (char)c;
            }
        }

        // --- 50 Hz position report ---
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_report_ms >= REPORT_INTERVAL_MS) {
            last_report_ms += REPORT_INTERVAL_MS;

            float shoulder = read_pot_deg(ADC_CH_SHOULDER);
            float elbow    = read_pot_deg(ADC_CH_ELBOW);

            printf("POS shoulder:%.2f elbow:%.2f\n", shoulder, elbow);

            // Drift check — only once CMD has been received
            if (cmd_active) {
                float ds = shoulder - cmd_shoulder;
                float de = elbow    - cmd_elbow;
                float abs_ds = ds < 0.0f ? -ds : ds;
                float abs_de = de < 0.0f ? -de : de;
                if (abs_ds > DRIFT_THRESHOLD_DEG || abs_de > DRIFT_THRESHOLD_DEG) {
                    printf("DRIFT shoulder:%.2f elbow:%.2f\n", ds, de);
                }
            }
        }
    }
}
