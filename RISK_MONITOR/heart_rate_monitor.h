#ifndef HEART_RATE_MONITOR_H
#define HEART_RATE_MONITOR_H

#include <stdint.h>

// Sampling and window
#define HRM_SAMPLE_PERIOD_SEC  0.5f       // 0.5 s between samples
#define HRM_WINDOW_SIZE        10u        // 5 s window @ 0.5 s
#define HRM_INVALID_HR_BPM     0.0f       // 0 means "no valid reading"

// Heart rate monitor state
typedef struct {
    // Circular buffer of the last N raw heart rate samples (BPM).
    // 0 or negative means "no valid reading" at that time.
    float    samples[HRM_WINDOW_SIZE];

    // Next write index into samples[]
    uint16_t index;

    // How many samples are valid (<= HRM_WINDOW_SIZE)
    uint16_t count;

    // Last non-zero, valid heart rate (for instantaneous ROC)
    float    last_valid_hr;

    // Last computed instantaneous rate of change (BPM/s) — ΔBPM / 0.5 s
    float    last_instant_roc;

    // Last computed window (5 s) rate of change (BPM/s) — robust slope
    float    last_window_roc;
} HeartRateMonitor;

// Initialize all fields. Call once at startup.
void HRM_Init(HeartRateMonitor *hm);

// Feed one new heart rate sample (BPM). hr_bpm > 0 => valid.
// Updates internal cache and both rates of change.
void HRM_Update(HeartRateMonitor *hm, float hr_bpm);

// Get last instantaneous rate of change (BPM/s).
float HRM_GetInstantRoc(const HeartRateMonitor *hm);

// Get last 5 second window rate of change (BPM/s), computed robustly.
float HRM_GetWindowRoc(const HeartRateMonitor *hm);

// Get the most recent raw sample (may be 0 if last sample was invalid).
float HRM_GetLatestHr(const HeartRateMonitor *hm);

#endif // HEART_RATE_MONITOR_H
