#ifndef RISK_MONITOR_H
#define RISK_MONITOR_H

#include <stdint.h>
#include <stdbool.h>

// Number of recent samples to use in the moving average.
// Example: if you call RiskMonitor_Update() at 10 Hz and
// RISK_WINDOW_SIZE = 10, then the window is ~1 second long.
#define RISK_WINDOW_SIZE  10

// Eye-closure confidence (%) above which we IMMEDIATELY alert,
// independent of the moving-average score.
#define EYE_HARD_ALERT_CONF_PCT  70.0f

// Heart-rate *rate-of-change* (window_roc_bpm_per_s) hard alert (absolute BPM/s).
// If |window_roc_bpm_per_s| >= HR_ROC_HARD_ALERT_ABS, we immediately alert.
#define HR_ROC_HARD_ALERT_ABS  4.0f

// Heart-rate *rate-of-change* risk shaping (absolute BPM/s).
// <= HR_ROC_SAFE_ABS   -> no HR risk
// >= HR_ROC_DANGER_ABS -> max HR risk
#define HR_ROC_SAFE_ABS    1.0f
#define HR_ROC_DANGER_ABS  HR_ROC_HARD_ALERT_ABS

// Acceleration risk shaping.
// We assume baseline head accel magnitude ~1 g (upright).
// <= ACCEL_SAFE_G   -> no accel risk
// >= ACCEL_DANGER_G -> max accel risk (strong drowsy nod / jerk)
#define ACCEL_BASE_G     1.0f
#define ACCEL_SAFE_G     1.3f   // small motion / light nodding
#define ACCEL_DANGER_G   1.8f   // typical drowsy head nod / strong jerk

// Structure holding internal state for the fused moving average.
// last_score is always in [0, 1].
typedef struct {
    // Circular buffer for the fused risk score
    float    window[RISK_WINDOW_SIZE];
    uint16_t index;       // write index into window
    uint16_t count;       // number of valid samples (<= RISK_WINDOW_SIZE)
    float    sum;         // running sum of values inside the window

    // Baselines (kept mainly for reference / flexibility)
    float baseline_hr_bpm;   // not used directly in ROC mode
    float baseline_accel_g;
    float baseline_alpha;    // not used in current risk logic

    // Weights for each input feature.
    // Eye should be the largest, HR ROC and accel equal second.
    // Defaults (if you pass <=0) are:
    //   eye_weight   = 0.6
    //   hr_weight    = 0.2
    //   accel_weight = 0.2
    float eye_weight;
    float hr_weight;
    float accel_weight;

    // Threshold on the moving average. If score >= threshold, flag is true.
    // Recommended: 0.75f.
    float threshold;

    // Last computed moving-average score in [0, 1]
    float last_score;

    // Current alert flag (true => activate buzzer)
    bool  alert_flag;
} RiskMonitor;

// Initialize the monitor. Call this once at startup.
//
//  - threshold: risk score level that will trigger the alert flag (0..1).
//               Recommended: 0.75f.
//  - eye_weight: weight for eye-open/closed feature (should be the largest).
//  - hr_weight: weight for heart-rate rate-of-change (window ROC).
//  - accel_weight: weight for head-movement changes.
//  - baseline_alpha: kept for compatibility, not used in current risk logic.
//  - initial_hr_bpm: starting baseline heart rate for the driver (not
//                    used directly in ROC mode, kept for compatibility).
//  - initial_accel_g: starting baseline acceleration magnitude (usually ~1g).
void RiskMonitor_Init(RiskMonitor *rm,
                      float threshold,
                      float eye_weight,
                      float hr_weight,
                      float accel_weight,
                      float baseline_alpha,
                      float initial_hr_bpm,
                      float initial_accel_g);

// Feed one new sample into the monitor.
//
//  - eye_closed_conf_pct: eye-closed confidence from 0–100
//      0   => model is sure eyes are open (no risk)
//      100 => model is sure eyes are closed (max risk)
//  - hr_window_roc_bpm_per_s: heart-rate window rate-of-change in BPM/s
//      (e.g., output of HRM_GetWindowRoc). Both positive and negative
//      spikes are treated using absolute value.
//  - accel_g: magnitude of head acceleration in units of g.
//
// Returns the current alert flag. If true, you should turn on the buzzer.
bool RiskMonitor_Update(RiskMonitor *rm,
                        float eye_closed_conf_pct,
                        float hr_window_roc_bpm_per_s,
                        float accel_g);

// Get the current moving-average risk score (0..1).
float RiskMonitor_GetScore(const RiskMonitor *rm);

// Get the current alert flag without updating.
bool RiskMonitor_GetAlertFlag(const RiskMonitor *rm);

#endif // RISK_MONITOR_H
