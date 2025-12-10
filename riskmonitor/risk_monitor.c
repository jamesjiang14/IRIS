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

    // Clear short eye window (~2-second eye history)
    for (uint16_t i = 0; i < EYE_WINDOW_SIZE; ++i) {
        rm->eye_window[i] = 0.0f;
    }
    rm->eye_index   = 0;
    rm->eye_count   = 0;
    rm->eye_sum     = 0.0f;
    rm->eye_avg_pct = 0.0f;

    // Clear fused-risk history buffer (not used for gating)
    for (uint16_t i = 0; i < RISK_WINDOW_SIZE; ++i) {
        rm->window[i] = 0.0f;
    }
    rm->index = 0;
    rm->count = 0;
    rm->sum   = 0.0f;

    // Baselines
    rm->baseline_hr_bpm  = (initial_hr_bpm  > 0.0f) ? initial_hr_bpm  : 70.0f;
    // New accel baseline is your XZ magnitude baseline ~0.7 g
    rm->baseline_accel_g = (initial_accel_g > 0.0f) ? initial_accel_g : 0.7f;
    rm->baseline_alpha   = baseline_alpha; // not used currently

    // Default weights if not provided
    if (eye_weight   <= 0.0f) eye_weight   = 0.6f;
    if (hr_weight    <= 0.0f) hr_weight    = 0.2f;
    if (accel_weight <= 0.0f) accel_weight = 0.2f;

    rm->eye_weight   = eye_weight;
    rm->hr_weight    = hr_weight;
    rm->accel_weight = accel_weight;

    // Default threshold if not provided: 0.15
    if (threshold <= 0.0f) {
        threshold = 0.15f;
    }
    rm->threshold  = clamp01(threshold);
    rm->last_score = 0.0f;
    rm->alert_flag = false;
}

// Map eye-closed confidence percentage (0–100) *window average* to a 0..1 risk feature.
static float compute_eye_feature(float eye_closed_conf_window_avg_pct)
{
    const float START   = 50.0f;  // start of eye-risk region
    const float MAX_PCT = 100.0f; // fully closed

    float c = eye_closed_conf_window_avg_pct;

    // Clamp weird values
    if (c <= 0.0f) {
        return 0.0f;
    }
    if (c >= 100.0f) {
        c = 100.0f;
    }

    // Below START: no eye risk
    if (c <= START) {
        return 0.0f;
    }

    // Between START and MAX: map to [0,1] linearly
    float x = (c - START) / (MAX_PCT - START);  // (50..100) -> (0..1)
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;

    return x;
}

// Heart-rate *rate-of-change* risk based on absolute BPM/s.
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

    float feature = (roc_abs - HR_ROC_SAFE_ABS) /
                    (HR_ROC_DANGER_ABS - HR_ROC_SAFE_ABS);
    return clamp01(feature);
}

/*
 * New accelerometer feature:
 * accel_g is the XZ magnitude from main (already computed there).
 *
 * Requirements:
 *   baseline around 0.7 g
 *   anything < 0.53 g => automatic buzzer
 *   anything > 0.85 g => automatic buzzer
 *
 * Implementation:
 *   - Treat the safe region as [0.53, 0.85] around the baseline 0.7.
 *   - Map distance from baseline to a 0..1 risk, so that exactly 0.53 or 0.85
 *     corresponds to max risk (1.0) and 0.7 corresponds to zero risk.
 */
static float compute_accel_feature(RiskMonitor *rm, float accel_g)
{
    const float BASE   = 0.7f;  // baseline XZ magnitude
    const float LOW    = 0.53f; // low threshold
    const float HIGH   = 0.85f; // high threshold

    float a = accel_g;
    if (a <= 0.0f) {
        // fall back to baseline if passed a weird value
        a = (rm->baseline_accel_g > 0.0f) ? rm->baseline_accel_g : BASE;
    }

    // Distance-based risk, symmetric around BASE
    float dist;
    if (a < BASE) {
        if (a <= LOW) {
            return 1.0f; // fully risky below/at LOW
        }
        dist = (BASE - a) / (BASE - LOW);  // LOW..BASE -> 1..0
    } else {
        if (a >= HIGH) {
            return 1.0f; // fully risky above/at HIGH
        }
        dist = (a - BASE) / (HIGH - BASE); // BASE..HIGH -> 0..1
    }

    float feature = dist; // already in [0,1]
    return clamp01(feature);
}

bool RiskMonitor_Update(RiskMonitor *rm,
                        float eye_closed_conf_pct,
                        float hr_window_roc_bpm_per_s,
                        float accel_g)
{
    if (rm == NULL) return false;

    // 1) Update short eye window (~2-second history at 0.5-second sampling)
    rm->eye_sum -= rm->eye_window[rm->eye_index];
    rm->eye_window[rm->eye_index] = eye_closed_conf_pct;
    rm->eye_sum += eye_closed_conf_pct;

    rm->eye_index++;
    if (rm->eye_index >= EYE_WINDOW_SIZE) {
        rm->eye_index = 0;
    }

    if (rm->eye_count < EYE_WINDOW_SIZE) {
        rm->eye_count++;
    }

    if (rm->eye_count > 0) {
        rm->eye_avg_pct = rm->eye_sum / (float)rm->eye_count;
    } else {
        rm->eye_avg_pct = eye_closed_conf_pct;
    }

    float eye_window_avg_pct = rm->eye_avg_pct;

    // 2) Convert raw inputs into normalized 0..1 features.
    //    Eye feature uses the 2-second window average.
    float eye_feature   = compute_eye_feature(eye_window_avg_pct);
    float hr_feature    = compute_hr_feature(rm, hr_window_roc_bpm_per_s);
    float accel_feature = compute_accel_feature(rm, accel_g); // now uses XZ mag

    // 3) Fuse into single instantaneous risk value in [0,1].
    float weight_sum = rm->eye_weight + rm->hr_weight + rm->accel_weight;
    if (weight_sum <= 0.0f) {
        weight_sum = 1.0f; // avoid divide-by-zero
    }

    float fused = (rm->eye_weight   * eye_feature +
                   rm->hr_weight    * hr_feature +
                   rm->accel_weight * accel_feature) / weight_sum;

    fused = clamp01(fused);

    // 4) Update history buffer (optional moving-average info).
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

    // 5) Store instantaneous fused risk as the "score" and decide alert flag.
    rm->last_score = fused;
    rm->alert_flag = (rm->last_score >= rm->threshold);

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
