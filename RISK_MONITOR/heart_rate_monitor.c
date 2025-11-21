#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include "heart_rate_monitor.h"

// ---------- Small helpers (window management) ----------

// Collect valid (>0) samples from circular buffer in chronological order
// (oldest -> newest). Returns count (<= HRM_WINDOW_SIZE). Copies into y[].
static uint16_t HRM_CollectValid(const HeartRateMonitor *hm, float y[], uint16_t maxn)
{
    if (!hm || maxn == 0) return 0;
    if (hm->count == 0)   return 0;

    // Oldest index: if buffer not yet full, oldest at 0; else next write is oldest
    uint16_t oldest = (hm->count < HRM_WINDOW_SIZE) ? 0u : hm->index;

    uint16_t out = 0u;
    uint16_t steps = hm->count; // how many chronological slots to read
    uint16_t idx = oldest;

    for (uint16_t i = 0; i < steps && out < maxn; ++i) {
        float v = hm->samples[idx];
        if (v > 0.0f) {
            y[out++] = v;
        }
        idx++;
        if (idx >= HRM_WINDOW_SIZE) idx = 0u;
    }
    return out;
}

// Compute median of array a[0..n-1]. n is small (<=10 or <=45), so simple sort is fine.
static float HRM_Median(float *a, uint16_t n)
{
    if (n == 0) return 0.0f;
    // Simple insertion sort (n is tiny)
    for (uint16_t i = 1; i < n; ++i) {
        float key = a[i];
        int j = (int)i - 1;
        while (j >= 0 && a[j] > key) {
            a[j + 1] = a[j];
            --j;
        }
        a[j + 1] = key;
    }
    if (n & 1u) {
        return a[n / 2u];
    } else {
        return 0.5f * (a[n / 2u - 1u] + a[n / 2u]);
    }
}

// Robust window slope (BPM/s) via Theil–Sen estimator (median of pairwise slopes).
// Uses all valid samples in the last 5 s window. With at most 10 samples this is cheap.
static float HRM_TheilSenWindowRoc(const HeartRateMonitor *hm)
{
    if (!hm || hm->count <= 1u) return 0.0f;

    float y[HRM_WINDOW_SIZE];
    uint16_t n = HRM_CollectValid(hm, y, HRM_WINDOW_SIZE);
    if (n <= 1u) return 0.0f;

    // Build all pairwise slopes (j>i)
    // Max pairs for n=10 is 45.
    float slopes[45];
    uint16_t m = 0u;

    for (uint16_t i = 0; i + 1u < n; ++i) {
        for (uint16_t j = i + 1u; j < n; ++j) {
            float dt = (float)(j - i) * HRM_SAMPLE_PERIOD_SEC; // seconds
            if (dt <= 0.0f) continue;
            float s = (y[j] - y[i]) / dt; // BPM per second
            slopes[m++] = s;
        }
    }
    if (m == 0u) return 0.0f;

    // Median of slopes
    return HRM_Median(slopes, m);
}

// ---------- Public API ----------

void HRM_Init(HeartRateMonitor *hm)
{
    if (!hm) return;
    for (uint16_t i = 0; i < HRM_WINDOW_SIZE; ++i) {
        hm->samples[i] = HRM_INVALID_HR_BPM;
    }
    hm->index            = 0u;
    hm->count            = 0u;
    hm->last_valid_hr    = 0.0f;
    hm->last_instant_roc = 0.0f;
    hm->last_window_roc  = 0.0f;
}

void HRM_Update(HeartRateMonitor *hm, float hr_bpm)
{
    if (!hm) return;

    // 1) Instantaneous ROC (vs previous valid reading)
    float instant_roc = 0.0f;
    if (hr_bpm > 0.0f && hm->last_valid_hr > 0.0f) {
        instant_roc = (hr_bpm - hm->last_valid_hr) / HRM_SAMPLE_PERIOD_SEC; // ΔBPM / 0.5 s
    }

    // 2) Push new sample into circular buffer
    hm->samples[hm->index] = hr_bpm;
    hm->index++;
    if (hm->index >= HRM_WINDOW_SIZE) hm->index = 0u;
    if (hm->count < HRM_WINDOW_SIZE)  hm->count++;

    // 3) Robust window ROC via Theil–Sen (immune to one-tick glitches)
    float window_roc = 0.0f;
    if (hr_bpm > 0.0f) {
        window_roc = HRM_TheilSenWindowRoc(hm);
        hm->last_valid_hr = hr_bpm;  // track latest valid raw reading
    }

    // 4) Publish
    hm->last_instant_roc = instant_roc;
    hm->last_window_roc  = window_roc;
}

float HRM_GetInstantRoc(const HeartRateMonitor *hm)
{
    return (hm ? hm->last_instant_roc : 0.0f);
}

float HRM_GetWindowRoc(const HeartRateMonitor *hm)
{
    return (hm ? hm->last_window_roc : 0.0f);
}

float HRM_GetLatestHr(const HeartRateMonitor *hm)
{
    if (!hm || hm->count == 0u) return 0.0f;
    uint16_t idx = (hm->index == 0u) ? (HRM_WINDOW_SIZE - 1u) : (hm->index - 1u);
    return hm->samples[idx];
}
