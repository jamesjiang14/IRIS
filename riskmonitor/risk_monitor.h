#ifndef RISK_MONITOR_H
#define RISK_MONITOR_H

#include <stdint.h>
#include <stdbool.h>

// Number of fused-risk samples kept in an internal history buffer.
// We no longer use this for gating the alert; it is kept only for
// optional smoothing/diagnostics.
#define RISK_WINDOW_SIZE   10

// Number of eye-closure samples in the short eye window.
// At 0.5 s per sample, 5 samples ≈ 2–2.5 seconds.
#define EYE_WINDOW_SIZE    5

// Eye-closure confidence (%) window average above which we *could* define
// a hard alert. Currently, we do NOT use this for hard alerts; we only
// trigger the buzzer when score >= threshold.
#define EYE_HARD_ALERT_CONF_PCT  70.0f

// Heart-rate *rate-of-change* (window_roc_bpm_per_s) hard alert (absolute BPM/s).
// Currently not used for gating the alert; kept for future expansion.
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

// Structure holding internal state for the fused risk monitor.
//
// IMPORTANT CHANGE:
// - last_score is now the *instantaneous* fused risk for the most recent
//   sample, in [0,1]. This is what you print as "score" and what is
//   compared against 'threshold' to decide the buzzer.
// - The internal window[] buffer is still maintained but is not used
//   for gating the buzzer (only for potential diagnostics if needed).
typedef struct {
    // --- Short-term eye window (~2 s at 0.5 s per sample) ---
    // Stores raw eye-closed confidence percentages (0–100) for the last
    // EYE_WINDOW_SIZE samples.
    float   eye_window[EYE_WINDOW_SIZE];
    uint8_t eye_index;      // write index into eye_window
    uint8_t eye_count;      // number of valid eye samples (<= EYE_WINDOW_SIZE)
    float   eye_sum;        // running sum of eye_window[] values
    float   eye_avg_pct;    // last eye-window average (0..100), for debug

    // --- Fused risk history buffer (not used for gating) --------------
    // Circular buffer for fused risk samples in [0,1].
    float    window[RISK_WINDOW_SIZE];
    uint16_t index;         // write index into window
    uint16_t count;         // number of valid samples (<= RISK_WINDOW_SIZE)
    float    sum;           // running sum of values inside window

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

    // Threshold on the fused risk score.
    // If last_score >= threshold, alert_flag is set true.
    // Recommended for current eye-only testing: 0.15f.
    float threshold;

    // Last fused risk score (instantaneous, 0..1) for the most recent sample.
    float last_score;

    // Current alert flag (true => activate buzzer)
    bool  alert_flag;
} RiskMonitor;

// Initialize the monitor. Call this once at startup.
//
//  - threshold: risk score level that will trigger the alert flag (0..1).
//               Recommended: 0.15f for your tests.
//  - eye_weight: weight for eye-open/closed feature (should be largest).
//  - hr_weight: weight for heart-rate rate-of-change (window ROC).
//  - accel_weight: weight for head-movement changes.
//  - baseline_alpha: kept for compatibility, not used in current risk logic.
//  - initial_hr_bpm: starting baseline heart rate for the driver (not
//                    used directly in ROC mode, kept for flexibility).
//  - initial_accel_g: starting baseline acceleration magnitude (usually ~1g).
void RiskMonitor_Init(RiskMonitor *rm,
                      float threshold,
                      float eye_weight,
                      float hr_weight,
                      float accel_weight,
                      float baseline_alpha,
                      float initial_hr_bpm,
                      float initial_accel_g);

// Feed one new eye/HR/accel sample into the monitor.
//
//  - eye_closed_conf_pct: eye-closed confidence from 0–100
//      0   => model is sure eyes are open (no risk)
//      100 => model is sure eyes are closed (max risk)
//
//      Internally, a ~2-second eye window (5 samples @0.5 s) is maintained.
//      The *window average* drives the eye risk so that short blinks do not
//      immediately trigger an alert. With the current tuning, you need
//      roughly ~2 seconds of sustained high closed-confidence to trigger.
//
//  - hr_window_roc_bpm_per_s: heart-rate window rate-of-change in BPM/s
//      (e.g., output of HRM_GetWindowRoc). Both positive and negative
//      spikes are treated using absolute value. This is currently included
//      in the fused risk but you are passing a dummy 0.0f, so it does not
//      affect the score.
//
//  - accel_g: magnitude of head acceleration in units of g.
//      Currently you are passing 1.0f, which maps to zero accel risk.
//
// Returns the current alert flag. If true, you should turn on the buzzer.
bool RiskMonitor_Update(RiskMonitor *rm,
                        float eye_closed_conf_pct,
                        float hr_window_roc_bpm_per_s,
                        float accel_g);

// Get the current fused risk score (instantaneous, 0..1) for the most
// recent sample.
float RiskMonitor_GetScore(const RiskMonitor *rm);

// Get the current alert flag without updating.
bool RiskMonitor_GetAlertFlag(const RiskMonitor *rm);

#endif // RISK_MONITOR_H
