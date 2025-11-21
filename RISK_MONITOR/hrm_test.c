#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "heart_rate_monitor.h"

#define INPUT_FILE  "heart_rate.txt"
#define OUTPUT_FILE "heart_rate_with_roc.txt"

int main(void)
{
    FILE *in = fopen(INPUT_FILE, "r");
    if (!in) {
        perror("Failed to open input file " INPUT_FILE);
        return 1;
    }

    FILE *out = fopen(OUTPUT_FILE, "w");
    if (!out) {
        perror("Failed to open output file " OUTPUT_FILE);
        fclose(in);
        return 1;
    }

    HeartRateMonitor hm;
    HRM_Init(&hm);

    char line[256];

    // Skip header line: "time_s,hr_bpm,scenario"
    if (fgets(line, sizeof(line), in) == NULL) {
        fprintf(stderr, "Input file is empty.\n");
        fclose(in);
        fclose(out);
        return 1;
    }

    // Write header for output file
    fprintf(out, "time_s\thr_bpm\tscenario\tinstant_roc_bpm_per_s\twindow_roc_bpm_per_s\n");
    fprintf(out, "--------------------------------------------------------------------------\n");

    while (fgets(line, sizeof(line), in)) {
        double time_s = 0.0;
        double hr_bpm_d = 0.0;
        char scenario[64] = {0};

        // Parse CSV line: time_s,hr_bpm,scenario
        int n = sscanf(line, " %lf , %lf , %63s", &time_s, &hr_bpm_d, scenario);
        if (n < 3) {
            // Ignore malformed lines
            continue;
        }

        float hr_bpm = (float)hr_bpm_d;

        // Update heart rate monitor state
        HRM_Update(&hm, hr_bpm);

        // Get instantaneous and 5-second window ROC (BPM/s)
        float inst = HRM_GetInstantRoc(&hm);
        float win  = HRM_GetWindowRoc(&hm);

        // Write a row to the output text file
        fprintf(out, "%.1f\t%.1f\t%s\t%.3f\t%.3f\n",
                (float)time_s, hr_bpm, scenario, inst, win);
    }

    fclose(in);
    fclose(out);

    printf("Processed heart rates from %s -> %s\n", INPUT_FILE, OUTPUT_FILE);
    return 0;
}
