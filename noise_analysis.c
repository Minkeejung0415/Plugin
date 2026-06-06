/*
 * noise_analysis.c — Real-time sensor noise and drift characterisation.
 *
 * See noise_analysis.h for algorithm description and usage notes.
 */

#include "noise_analysis.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

/* -------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------- */

static void axis_stats_init(AxisStats *s)
{
    s->n       = 0;
    s->mean    = 0.0;
    s->M2      = 0.0;
    s->min_val = 1e30;
    s->max_val = -1e30;
}

/* Welford's online update: O(1), numerically stable. */
static void axis_stats_push(AxisStats *s, double x)
{
    s->n++;
    double delta  = x - s->mean;
    s->mean      += delta / (double)s->n;
    double delta2 = x - s->mean;
    s->M2        += delta * delta2;

    if (x < s->min_val) s->min_val = x;
    if (x > s->max_val) s->max_val = x;
}

/* Population variance (unbiased sample variance when n > 1). */
static double axis_stats_variance(const AxisStats *s)
{
    if (s->n < 2) return 0.0;
    return s->M2 / (double)(s->n - 1);
}

static double axis_stats_stddev(const AxisStats *s)
{
    return sqrt(axis_stats_variance(s));
}

/* --- Rolling window ---------------------------------------------------- */

static void rolling_init(RollingWindow *w)
{
    memset(w->buf, 0, sizeof(w->buf));
    w->head   = 0;
    w->filled = 0;
}

static void rolling_push(RollingWindow *w, float value)
{
    w->buf[w->head] = value;
    w->head = (w->head + 1) % NOISE_RECENT_WINDOW;
    if (w->filled < NOISE_RECENT_WINDOW)
        w->filled++;
}

static float rolling_mean(const RollingWindow *w)
{
    if (w->filled == 0) return 0.0f;
    double sum = 0.0;
    for (uint32_t i = 0; i < w->filled; i++)
        sum += (double)w->buf[i];
    return (float)(sum / (double)w->filled);
}

/* --- Drift tracker ----------------------------------------------------- */

static void drift_init(DriftTracker *d)
{
    d->early_mean  = 0.0f;
    d->early_count = 0;
    d->early_sum   = 0.0;
    rolling_init(&d->recent);
    d->recent_mean = 0.0f;
}

static void drift_push(DriftTracker *d, float value)
{
    /* Accumulate early baseline */
    if (d->early_count < NOISE_EARLY_WINDOW) {
        d->early_sum += (double)value;
        d->early_count++;
        if (d->early_count == NOISE_EARLY_WINDOW)
            d->early_mean = (float)(d->early_sum / (double)NOISE_EARLY_WINDOW);
    }

    rolling_push(&d->recent, value);
    d->recent_mean = rolling_mean(&d->recent);
}

/* Signed drift from early baseline; valid only once early window is full. */
static float drift_delta(const DriftTracker *d)
{
    if (d->early_count < NOISE_EARLY_WINDOW) return 0.0f;
    return d->recent_mean - d->early_mean;
}


/* =========================================================================
 * Public API
 * ========================================================================= */

void noise_analyzer_init(
    NoiseAnalyzer *na,
    float          sample_rate_hz,
    bool           has_mag,
    float          accel_mps2_per_lsb,
    float          gyro_dps_per_lsb)
{
    memset(na, 0, sizeof(*na));
    na->sample_rate_hz      = (sample_rate_hz > 0.0f) ? sample_rate_hz : 1.0f;
    na->has_mag             = has_mag;
    na->num_axes            = has_mag ? 9 : 6;
    na->accel_mps2_per_lsb  = (accel_mps2_per_lsb > 0.0f) ? accel_mps2_per_lsb : 1.0f;
    na->gyro_dps_per_lsb    = (gyro_dps_per_lsb   > 0.0f) ? gyro_dps_per_lsb   : 1.0f;
    na->initialized         = true;

    for (int i = 0; i < NOISE_MAX_AXES; i++) {
        axis_stats_init(&na->stats[i]);
        drift_init(&na->drift[i]);
    }
}

void noise_analyzer_reset(NoiseAnalyzer *na)
{
    bool has_mag            = na->has_mag;
    float sample_rate_hz    = na->sample_rate_hz;
    float accel_scale       = na->accel_mps2_per_lsb;
    float gyro_scale        = na->gyro_dps_per_lsb;
    noise_analyzer_init(na, sample_rate_hz, has_mag, accel_scale, gyro_scale);
}

void noise_analyzer_update(
    NoiseAnalyzer  *na,
    const int16_t   raw[NOISE_MAX_AXES],
    bool            mag_is_fresh)
{
    if (!na || !na->initialized || !raw) return;

    na->total_samples++;

    /* Accel axes 0,1,2 */
    for (int i = 0; i < 3; i++) {
        float phys = (float)raw[i] * na->accel_mps2_per_lsb;
        axis_stats_push(&na->stats[i], (double)phys);
        drift_push(&na->drift[i], phys);
    }

    /* Gyro axes 3,4,5 */
    for (int i = 3; i < 6; i++) {
        float phys = (float)raw[i] * na->gyro_dps_per_lsb;
        axis_stats_push(&na->stats[i], (double)phys);
        drift_push(&na->drift[i], phys);
    }

    /* Magnetometer axes 6,7,8 — only when fresh */
    if (na->has_mag && mag_is_fresh) {
        for (int i = 6; i < 9; i++) {
            float phys = (float)raw[i];  /* mag kept in raw LSB; caller can scale */
            axis_stats_push(&na->stats[i], (double)phys);
            drift_push(&na->drift[i], phys);
        }
    }
}

/* Noise density helper — converts stddev to spectral density */
float noise_analyzer_gyro_arw(const NoiseAnalyzer *na, int axis)
{
    if (!na || axis < 0 || axis > 2) return 0.0f;
    double stddev = axis_stats_stddev(&na->stats[3 + axis]);
    /* ARW = stddev / sqrt(fs) in °/s/√Hz */
    return (float)(stddev / sqrt((double)na->sample_rate_hz));
}

float noise_analyzer_accel_vrw(const NoiseAnalyzer *na, int axis)
{
    if (!na || axis < 0 || axis > 2) return 0.0f;
    double stddev = axis_stats_stddev(&na->stats[axis]);
    return (float)(stddev / sqrt((double)na->sample_rate_hz));
}

bool noise_analyzer_gyro_drift_exceeded(const NoiseAnalyzer *na)
{
    if (!na || !na->initialized) return false;
    for (int i = 0; i < 3; i++) {
        double stddev = axis_stats_stddev(&na->stats[3 + i]);
        float  drift  = fabsf(drift_delta(&na->drift[3 + i]));
        if (stddev > 0.0 && drift > NOISE_DRIFT_ALERT_FACTOR * (float)stddev)
            return true;
    }
    return false;
}

void noise_analyzer_report(
    const NoiseAnalyzer *na,
    const char          *sensor_name,
    FILE                *fp)
{
    if (!na || !fp) return;
    if (!sensor_name) sensor_name = "?";

    const char *axis_labels[9] = {
        "ax(m/s²)", "ay(m/s²)", "az(m/s²)",
        "gx(°/s)",  "gy(°/s)",  "gz(°/s)",
        "mx(LSB)",  "my(LSB)",  "mz(LSB)"
    };

    fprintf(fp,
            "\n── Noise Report: %s  (%.0f Hz, %llu samples) ──────────────\n",
            sensor_name,
            (double)na->sample_rate_hz,
            (unsigned long long)na->total_samples);

    if (na->bus_error_count > 0)
        fprintf(fp, "  WARNING: %llu bus read errors during this session\n",
                (unsigned long long)na->bus_error_count);

    fprintf(fp, "  %-12s  %10s  %10s  %10s  %10s  %10s  %9s\n",
            "axis", "mean", "std_dev", "min", "max", "drift", "density");
    fprintf(fp, "  %-12s  %10s  %10s  %10s  %10s  %10s  %9s\n",
            "────────────", "──────────", "──────────",
            "──────────", "──────────", "──────────", "─────────");

    int n_axes = na->has_mag ? 9 : 6;
    for (int i = 0; i < n_axes; i++) {
        const AxisStats    *s = &na->stats[i];
        const DriftTracker *d = &na->drift[i];

        if (s->n == 0) continue;

        double stddev = axis_stats_stddev(s);

        /* Noise density differs by axis type */
        float density = 0.0f;
        const char *density_unit = "";
        if (i < 3) {
            density      = noise_analyzer_accel_vrw(na, i);
            density_unit = " m/s²/√Hz";
        } else if (i < 6) {
            density      = noise_analyzer_gyro_arw(na, i - 3);
            density_unit = " °/s/√Hz";
        }

        float d_val = (d->early_count >= NOISE_EARLY_WINDOW)
                      ? drift_delta(d) : 0.0f;
        const char *drift_note = (d->early_count < NOISE_EARLY_WINDOW)
                                 ? " (warming up)" : "";

        fprintf(fp,
                "  %-12s  %10.4f  %10.6f  %10.4f  %10.4f  %+10.5f%s  %9.6f%s\n",
                axis_labels[i],
                s->mean,
                stddev,
                s->min_val,
                s->max_val,
                (double)d_val, drift_note,
                (double)density,
                density_unit);
    }

    /* Gyro drift alarm */
    if (noise_analyzer_gyro_drift_exceeded(na))
        fprintf(fp,
                "  *** GYRO DRIFT ALERT: bias shift > %.0fx noise floor ***\n",
                (double)NOISE_DRIFT_ALERT_FACTOR);

    fprintf(fp, "──────────────────────────────────────────────────────────────\n");
    fflush(fp);
}

void noise_analyzer_csv_row(
    const NoiseAnalyzer *na,
    const char          *sensor_name,
    FILE                *fp)
{
    if (!na || !fp) return;
    if (!sensor_name) sensor_name = "?";

    fprintf(fp, "%s,%llu",
            sensor_name,
            (unsigned long long)na->total_samples);

    /* stddev columns: ax ay az gx gy gz */
    for (int i = 0; i < 6; i++)
        fprintf(fp, ",%.6f", axis_stats_stddev(&na->stats[i]));

    /* drift columns: ax ay az gx gy gz */
    for (int i = 0; i < 6; i++) {
        float d = (na->drift[i].early_count >= NOISE_EARLY_WINDOW)
                  ? drift_delta(&na->drift[i]) : 0.0f;
        fprintf(fp, ",%+.6f", (double)d);
    }

    /* bus errors */
    fprintf(fp, ",%llu\n", (unsigned long long)na->bus_error_count);
}
