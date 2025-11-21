#include "risk_monitor.h"

#include <math.h>
#include <stddef.h>

static float clamp01(float x)
{
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

void RiskMonitor_Init(RiskMonitor *rm,
                      float threshold,
                      float eye_weight,
                      float hr_weight,
                      float accel_weight,
                      float baseline_alpha,
                      float initial_hr_bpm,
                      float initial_accel_g)
{
    if (rm == NULL) return;

    for (uint16_t i = 0; i < RISK_WINDOW_SIZE; ++i) {
        rm->window[i] = 0.0f;
    }

    rm->index = 0;
    rm->count = 0;
    rm->sum   = 0.0f;

    rm->baseline_hr_bpm  = (initial_hr_bpm  > 0.0f) ? initial_hr_bpm  : 70.0f;
    rm->baseline_accel_g = (initial_accel_g > 0.0f) ? initial_accel_g : ACCEL_BASE_G;
    rm->baseline_alpha   = baseline_alpha; // not used right now

    // Default weights if not provided:
    // Eye heaviest, HR ROC and accel equal second.
    if (eye_weight   <= 0.0f) eye_weight   = 0.6f;
    if (hr_weight    <= 0.0f) hr_weight    = 0.2f;
    if (accel_weight <= 0.0f) accel_weight = 0.2f;

    rm->eye_weight   = eye_weight;
    rm->hr_weight    = hr_weight;
    rm->accel_weight = accel_weight;

    // Default threshold if not provided: 0.75
    if (threshold <= 0.0f) {
        threshold = 0.75f;
    }
    rm->threshold  = clamp01(threshold);
    rm->last_score = 0.0f;
    rm->alert_flag = false;
}

// Map eye-closed confidence percentage (0–100) to a 0..1 risk feature,
// but only start counting risk once eyes are substantially closed.
//
//  - <= 60% closed  => 0 risk
//  - 60–100% closed => ramps 0 -> 1 with quadratic scaling
static float compute_eye_feature(float eye_closed_conf_pct)
{
    const float START = 60.0f;   // start of eye-risk region
    const float MAX   = 100.0f;  // fully closed

    float c = eye_closed_conf_pct;

    // Clamp weird values
    if (c <= 0.0f) {
        return 0.0f;
    }
    if (c >= 100.0f) {
        return 1.0f;
    }

    // Below START: no eye risk
    if (c <= START) {
        return 0.0f;
    }

    // Between START and MAX: map to [0,1] then square to emphasize high closure
    float x = (c - START) / (MAX - START);  // (60..100) -> (0..1)
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;

    return x * x;
}

// Heart-rate *rate-of-change* risk based on absolute BPM/s:
// <= HR_ROC_SAFE_ABS   => 0 risk
// >= HR_ROC_DANGER_ABS => 1 risk
// linear in between.
static float compute_hr_feature(RiskMonitor *rm, float hr_window_roc_bpm_per_s)
{
    (void)rm; // unused for now

    float roc_abs = fabsf(hr_window_roc_bpm_per_s);
    if (roc_abs <= HR_ROC_SAFE_ABS) {
        return 0.0f;
    }
    if (roc_abs >= HR_ROC_DANGER_ABS) {
        return 1.0f;
    }

    float feature = (roc_abs - HR_ROC_SAFE_ABS) / (HR_ROC_DANGER_ABS - HR_ROC_SAFE_ABS);
    return clamp01(feature);
}

// Acceleration risk based on absolute g-level above baseline:
// <= ACCEL_SAFE_G   => 0 risk
// >= ACCEL_DANGER_G => 1 risk
// linear in between.
static float compute_accel_feature(RiskMonitor *rm, float accel_g)
{
    float a = accel_g;
    if (a <= 0.0f) {
        a = rm->baseline_accel_g;
    }

    // Safe & danger thresholds anchored to baseline
    float safe   = (rm->baseline_accel_g > 0.0f)
                   ? (rm->baseline_accel_g + (ACCEL_SAFE_G - ACCEL_BASE_G))
                   : ACCEL_SAFE_G;
    float danger = (rm->baseline_accel_g > 0.0f)
                   ? (rm->baseline_accel_g + (ACCEL_DANGER_G - ACCEL_BASE_G))
                   : ACCEL_DANGER_G;

    if (a <= safe) {
        return 0.0f;
    }
    if (a >= danger) {
        return 1.0f;
    }

    float feature = (a - safe) / (danger - safe);
    return clamp01(feature);
}

bool RiskMonitor_Update(RiskMonitor *rm,
                        float eye_closed_conf_pct,
                        float hr_window_roc_bpm_per_s,
                        float accel_g)
{
    if (rm == NULL) return false;

    // 1) Hard eye-closure alert.
    bool eye_hard_alert = (eye_closed_conf_pct >= EYE_HARD_ALERT_CONF_PCT);

    // 2) Hard heart-rate ROC alert based on absolute window ROC.
    float hr_roc_abs = fabsf(hr_window_roc_bpm_per_s);
    bool hr_roc_hard_alert = (hr_roc_abs >= HR_ROC_HARD_ALERT_ABS);

    // 3) Convert raw inputs into normalized 0..1 features.
    float eye_feature   = compute_eye_feature(eye_closed_conf_pct);
    float hr_feature    = compute_hr_feature(rm, hr_window_roc_bpm_per_s);
    float accel_feature = compute_accel_feature(rm, accel_g);

    // 4) Fuse into single instantaneous risk value in [0,1].
    float weight_sum = rm->eye_weight + rm->hr_weight + rm->accel_weight;
    if (weight_sum <= 0.0f) {
        weight_sum = 1.0f; // avoid divide-by-zero
    }

    float fused = (rm->eye_weight   * eye_feature +
                   rm->hr_weight    * hr_feature +
                   rm->accel_weight * accel_feature) / weight_sum;

    // 5) Update moving average using circular buffer (Score in [0,1]).
    rm->sum -= rm->window[rm->index];
    rm->window[rm->index] = fused;
    rm->sum += fused;

    rm->index++;
    if (rm->index >= RISK_WINDOW_SIZE) {
        rm->index = 0;
    }

    if (rm->count < RISK_WINDOW_SIZE) {
        rm->count++;
    }

    if (rm->count > 0) {
        rm->last_score = rm->sum / (float)rm->count;  // in [0,1]
    } else {
        rm->last_score = fused;
    }

    // 6) Decide alert flag:
    //    - If eye_hard_alert or hr_roc_hard_alert => ALWAYS alert.
    //    - Else fall back to moving-average threshold.
    if (eye_hard_alert || hr_roc_hard_alert) {
        rm->alert_flag = true;
    } else if (rm->last_score >= rm->threshold) {
        rm->alert_flag = true;
    } else {
        rm->alert_flag = false;
    }

    return rm->alert_flag;
}

float RiskMonitor_GetScore(const RiskMonitor *rm)
{
    if (rm == NULL) return 0.0f;
    return rm->last_score;
}

bool RiskMonitor_GetAlertFlag(const RiskMonitor *rm)
{
    if (rm == NULL) return false;
    return rm->alert_flag;
}
