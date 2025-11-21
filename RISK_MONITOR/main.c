#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "risk_monitor.h"

#define REPEAT_SAMPLES  10  // how many times to apply each sample to fill the window

// Print header once to the given stream
static void print_header(FILE *out)
{
    fprintf(out, "Idx\tEyeConf\tHR_ROC\tAccel_g\tScore\tAlert\n");
    fprintf(out, "--------------------------------------------------------------\n");
}

int main(void)
{
    // Eye-closure confidence values to sweep (0–100)
    const float eye_vals[] = { 0.0f, 20.0f, 30.0f, 50.0f, 65.0f, 70.0f, 90.0f, 100.0f };
    const int   NUM_EYE    = (int)(sizeof(eye_vals) / sizeof(eye_vals[0]));

    // Heart-rate window ROC (BPM/s) to sweep.
    // Anything with |HR_ROC| >= 4.0f should trigger the hard HR ROC alert.
    const float HR_ROC_MIN  = 0.0f;
    const float HR_ROC_MAX  = 8.0f;
    const float HR_ROC_STEP = 0.5f;

    // Acceleration: 1.0–2.4 g in steps of 0.2
    const float ACCEL_MIN  = 1.0f;
    const float ACCEL_STEP = 0.2f;
    const int   NUM_ACCEL  = 8;   // 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4

    float baseline_hr    = 70.0f;  // kept for compatibility
    float baseline_accel = 1.0f;   // ~1 g when still/upright

    // Open output file
    const char *filename = "risk_sweep.txt";
    FILE *out = fopen(filename, "w");
    if (!out) {
        // Fallback to stdout if file open fails
        perror("Failed to open output file");
        out = stdout;
    }

    print_header(out);

    int idx = 0;

    for (int ei = 0; ei < NUM_EYE; ++ei) {
        float eye_closed_conf_pct = eye_vals[ei];

        for (float hr_roc = HR_ROC_MIN; hr_roc <= HR_ROC_MAX + 1e-6f; hr_roc += HR_ROC_STEP) {

            for (int ai = 0; ai < NUM_ACCEL; ++ai) {
                float accel_g = ACCEL_MIN + ACCEL_STEP * (float)ai;

                RiskMonitor rm;
                bool alert = false;

                // Fresh monitor for each combination so cases are independent
                RiskMonitor_Init(&rm,
                                 0.2f,  // threshold for alert (debugging; device can use ~0.75)
                                 0.6f,  // eye weight (highest)
                                 0.2f,  // heart-rate ROC weight
                                 0.2f,  // accelerometer weight
                                 0.05f, // baseline alpha (kept for compatibility)
                                 baseline_hr,
                                 baseline_accel);

                // Feed the same sample multiple times to let the moving average settle
                for (int k = 0; k < REPEAT_SAMPLES; ++k) {
                    alert = RiskMonitor_Update(
                        &rm,
                        eye_closed_conf_pct,
                        hr_roc,
                        accel_g
                    );
                }

                float score = RiskMonitor_GetScore(&rm);

                fprintf(out, "%4d\t%.1f\t%.2f\t%.2f\t%.3f\t%s\n",
                        idx,
                        eye_closed_conf_pct,
                        hr_roc,
                        accel_g,
                        score,
                        alert ? "YES" : "no");

                idx++;
            }
        }
    }

    if (out != stdout) {
        fclose(out);
        printf("Wrote %d rows to %s\n", idx, filename);
    }

    return 0;
}
