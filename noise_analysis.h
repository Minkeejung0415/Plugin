/*
 * noise_analysis.h — Real-time sensor noise and drift characterisation.
 *
 * Intended use
 * ------------
 * Attach one NoiseAnalyzer per IMU sensor slot.  Feed every raw sample
 * through noise_analyzer_update().  After a long-duration run, call
 * noise_analyzer_report() to print Allan-deviation-style statistics to
 * the terminal or a log file.
 *
 * Statistical methods
 * -------------------
 * - Welford's online algorithm:  numerically stable, single-pass mean and
 *   variance without accumulating large sums.
 * - Bias drift window:  a short "early" window is captured at startup;
 *   a rolling "recent" window tracks the current mean.  The difference
 *   between the two estimates quantifies long-term bias instability.
 * - Noise density:  standard deviation of the gyro signal normalised by
 *   sqrt(sample_rate_hz) gives ARW (angle random walk) in  °/s/√Hz.
 * - Velocity random walk (VRW) is similarly reported for the accelerometer.
 *
 * Axes
 * ----
 * Each NoiseAnalyzer holds independent statistics for:
 *   axes [0..2]  accelerometer   X, Y, Z
 *   axes [3..5]  gyroscope       X, Y, Z
 *   (axis [6..8] magnetometer, optional — only if has_mag == true)
 */

#ifndef NOISE_ANALYSIS_H
#define NOISE_ANALYSIS_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum axes tracked per sensor (accel + gyro + mag) */
#define NOISE_MAX_AXES 9

/* Samples held in the rolling "recent" window for drift estimation */
#define NOISE_RECENT_WINDOW 512

/* Samples collected during early startup for baseline bias capture */
#define NOISE_EARLY_WINDOW 200

/* Threshold multiplier: alert printed when bias drift > factor * noise_stddev */
#define NOISE_DRIFT_ALERT_FACTOR 3.0f


/* --- Per-axis online statistics (Welford) -------------------------------- */
typedef struct {
    uint64_t n;         /* Total sample count */
    double   mean;      /* Running mean */
    double   M2;        /* Sum of squared deviations (for variance) */
    double   min_val;
    double   max_val;
} AxisStats;


/* --- Circular buffer for rolling recent-window mean ---------------------- */
typedef struct {
    float    buf[NOISE_RECENT_WINDOW];
    int      head;
    uint32_t filled;    /* 0..NOISE_RECENT_WINDOW; capped once full */
} RollingWindow;


/* --- Per-axis drift tracker --------------------------------------------- */
typedef struct {
    float    early_mean;    /* Mean over first NOISE_EARLY_WINDOW samples */
    uint32_t early_count;   /* Samples collected so far into early window */
    double   early_sum;

    RollingWindow recent;
    float    recent_mean;   /* Most-recent rolling window mean */
} DriftTracker;


/* --- Top-level per-sensor analyzer -------------------------------------- */
typedef struct {
    bool     initialized;
    bool     has_mag;
    int      num_axes;       /* 6 (no mag) or 9 (with mag) */
    float    sample_rate_hz;
    uint64_t total_samples;
    uint64_t bus_error_count; /* Incremented by caller on each read error */

    /* Accel scale in m/s² per raw LSB (for physical-unit reporting) */
    float    accel_mps2_per_lsb;
    /* Gyro scale in °/s per raw LSB */
    float    gyro_dps_per_lsb;

    AxisStats    stats[NOISE_MAX_AXES];
    DriftTracker drift[NOISE_MAX_AXES];
} NoiseAnalyzer;


/* -------------------------------------------------------------------------
 * Lifecycle
 * ------------------------------------------------------------------------- */

/*
 * Initialises all fields of an analyzer.  Must be called before any
 * update or query.  Pass has_mag=true for 9-DOF sensors.
 * accel_mps2_per_lsb and gyro_dps_per_lsb convert raw counts to physical
 * units; pass 1.0f to keep everything in raw counts.
 */
void noise_analyzer_init(
    NoiseAnalyzer *na,
    float          sample_rate_hz,
    bool           has_mag,
    float          accel_mps2_per_lsb,
    float          gyro_dps_per_lsb
);

void noise_analyzer_reset(NoiseAnalyzer *na);


/* -------------------------------------------------------------------------
 * Per-sample update
 * ------------------------------------------------------------------------- */

/*
 * Feeds one raw sample into all statistics.
 *
 * raw[0..2]  accel  X, Y, Z  (raw int16 counts)
 * raw[3..5]  gyro   X, Y, Z
 * raw[6..8]  mag    X, Y, Z  (only consumed when has_mag && mag_is_fresh)
 *
 * mag_is_fresh should be true only when the magnetometer has a new reading;
 * stale mag samples must not be fed repeatedly or they will bias the stats.
 */
void noise_analyzer_update(
    NoiseAnalyzer  *na,
    const int16_t   raw[NOISE_MAX_AXES],
    bool            mag_is_fresh
);


/* -------------------------------------------------------------------------
 * Reporting
 * ------------------------------------------------------------------------- */

/*
 * Prints a formatted noise/drift report to fp (use stderr or stdout).
 * sensor_name is used only for the heading.
 */
void noise_analyzer_report(
    const NoiseAnalyzer *na,
    const char          *sensor_name,
    FILE                *fp
);

/*
 * Returns true if the absolute drift on any gyro axis (axes 3..5) exceeds
 * NOISE_DRIFT_ALERT_FACTOR * stddev_of_that_axis.  Useful for runtime
 * thermal compensation warnings.
 */
bool noise_analyzer_gyro_drift_exceeded(const NoiseAnalyzer *na);

/*
 * Writes a one-line CSV summary (no header) to fp.
 * Format: sensor_name,samples,ax_std,ay_std,az_std,gx_std,gy_std,gz_std,
 *         ax_drift,ay_drift,az_drift,gx_drift,gy_drift,gz_drift
 */
void noise_analyzer_csv_row(
    const NoiseAnalyzer *na,
    const char          *sensor_name,
    FILE                *fp
);

/* Returns current noise density in °/s/√Hz for one gyro axis (0, 1, or 2). */
float noise_analyzer_gyro_arw(const NoiseAnalyzer *na, int axis);

/* Returns current noise density in (m/s²)/√Hz for one accel axis (0, 1, or 2). */
float noise_analyzer_accel_vrw(const NoiseAnalyzer *na, int axis);

#ifdef __cplusplus
}
#endif

#endif /* NOISE_ANALYSIS_H */
