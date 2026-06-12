// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
//
// SPDX-License-Identifier: MIT
//
// Pure C conversion for Red Pitaya embedded use.
// Original C++ implementation by Daniel Laidig.

/*
 * ============================================================================
 *  VQF — Versatile Quaternion-based Filter
 * ============================================================================
 *
 *  WHAT THIS FILE IS:
 *  This header describes a single-sensor orientation filter. You feed it
 *  raw measurements from a gyroscope, accelerometer, and optionally a
 *  magnetometer, and it continuously tells you "which way is this sensor
 *  pointing?" as a quaternion (four numbers: w, x, y, z).
 *
 *  WHY QUATERNIONS?
 *  A quaternion is a compact, gimbal-lock-free way to represent a 3-D
 *  rotation. Think of it as four numbers that together encode the axis you
 *  rotated around and how far you rotated.  Euler angles (pitch/roll/yaw)
 *  are intuitive but break down in certain orientations (gimbal lock).
 *  Quaternions avoid that entirely.
 *
 *  THE THREE SENSOR ROLES:
 *    Gyroscope    → measures how fast you are spinning right now (rad/s).
 *                   Very accurate for short bursts but drifts over time.
 *    Accelerometer → measures gravity's pull direction (m/s²).
 *                   Tells you which way "down" is — corrects tilt drift.
 *    Magnetometer  → measures Earth's magnetic field.
 *                   Tells you which way is north — corrects yaw drift.
 *
 *  HOW VQF COMBINES THEM:
 *    1. Integrate gyro readings each time step → fast but drifty orientation
 *    2. Compare accelerometer "down" vector to filter's predicted "down"
 *       → gently pull the orientation back toward true vertical (tauAcc)
 *    3. (Optional) Compare magnetometer heading to filter's heading
 *       → gently pull yaw toward true north (tauMag)
 *    4. Estimate and subtract gyro bias so drift shrinks over time
 *    5. Detect when the sensor is still (rest) for faster, more accurate
 *       bias convergence
 *
 *  OUTPUT MODES:
 *    3D  — gyro strapdown only (raw integration, no correction)
 *    6D  — gyro + accel (tilt-corrected, no heading reference)
 *    9D  — gyro + accel + mag (full orientation with heading)
 *
 *  USAGE PATTERN:
 *    VQF vqf;
 *    vqf_init(&vqf, gyr_Ts, acc_Ts, mag_Ts);   // one-time setup
 *    // every sample:
 *    vqf_update(&vqf, gyr, acc);                // 6D update
 *    vqf_get_quat_6d(&vqf, out_quat);           // read result
 * ============================================================================
 */

#ifndef VQF_H
#define VQF_H

#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// #define VQF_SINGLE_PRECISION
// #define VQF_NO_MOTION_BIAS_ESTIMATION

/**
 * @brief Floating-point type used for most operations.
 *
 * By default double. Define VQF_SINGLE_PRECISION to use float instead.
 * Note: Butterworth filter internals always use double for numeric stability.
 */
#ifndef VQF_REAL_T_DEFINED
#define VQF_REAL_T_DEFINED
#ifndef VQF_SINGLE_PRECISION
typedef double vqf_real_t;
#else
typedef float vqf_real_t;
#endif
#endif

/*
 * ----------------------------------------------------------------------------
 *  VQFParams — Tuning knobs for the filter
 * ----------------------------------------------------------------------------
 *  All fields have sensible defaults (set by vqf_params_init).  You only
 *  need to change them if you know why.
 *
 *  The two most important tunables are tauAcc and tauMag:
 *    • Larger tauAcc  → smoother tilt correction, less reactive to real motion
 *    • Smaller tauAcc → faster tilt correction, but motion noise bleeds in
 *    • Same idea for tauMag with respect to heading correction
 * ----------------------------------------------------------------------------
 */
/**
 * @brief Tuning parameters for the VQF algorithm.
 */
typedef struct {
    /* How quickly to trust the accelerometer for tilt correction.
     * 3 s means the filter blends 63% of the accel correction over 3 s.
     * Increase if the body moves fast (reduces false tilt corrections).  */
    vqf_real_t tauAcc;                   /**< Accelerometer low-pass filter time constant (default: 3.0 s) */

    /* How quickly to trust the magnetometer for heading correction.
     * 9 s means slow, stable heading convergence.  Increase near metal.  */
    vqf_real_t tauMag;                   /**< Magnetometer update time constant (default: 9.0 s) */

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    bool motionBiasEstEnabled;           /**< Enable gyro bias estimation during motion (default: true) */
#endif
    bool restBiasEstEnabled;             /**< Enable rest detection and bias estimation (default: true) */
    bool magDistRejectionEnabled;        /**< Enable magnetic disturbance rejection (default: true) */

    vqf_real_t biasSigmaInit;            /**< Initial bias estimation uncertainty in deg/s (default: 0.5) */
    vqf_real_t biasForgettingTime;       /**< Time for bias uncertainty to increase from 0 to 0.1 deg/s (default: 100.0 s) */
    vqf_real_t biasClip;                 /**< Maximum expected gyro bias in deg/s (default: 2.0) */
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    vqf_real_t biasSigmaMotion;          /**< Converged motion bias uncertainty in deg/s (default: 0.1) */
    vqf_real_t biasVerticalForgettingFactor; /**< Forgetting factor for unobservable vertical bias (default: 0.0001) */
#endif
    vqf_real_t biasSigmaRest;            /**< Converged rest bias uncertainty in deg/s (default: 0.03) */

    vqf_real_t restMinT;                 /**< Minimum rest duration in seconds (default: 1.5) */
    vqf_real_t restFilterTau;            /**< Rest detection low-pass filter time constant (default: 0.5 s) */
    vqf_real_t restThGyr;                /**< Gyroscope rest detection threshold in deg/s (default: 2.0) */
    vqf_real_t restThAcc;                /**< Accelerometer rest detection threshold in m/s^2 (default: 0.5) */

    vqf_real_t magCurrentTau;            /**< Low-pass filter time constant for mag norm/dip (default: 0.05 s) */
    vqf_real_t magRefTau;                /**< Magnetic field reference adjustment time constant (default: 20.0 s) */
    vqf_real_t magNormTh;                /**< Relative threshold for mag norm disturbance (default: 0.1) */
    vqf_real_t magDipTh;                 /**< Dip angle threshold for mag disturbance in degrees (default: 10.0) */
    vqf_real_t magNewTime;               /**< Duration to accept new mag field reference (default: 20.0 s) */
    vqf_real_t magNewFirstTime;          /**< Duration for first mag field acceptance (default: 5.0 s) */
    vqf_real_t magNewMinGyr;             /**< Minimum angular velocity for mag acceptance (default: 20.0 deg/s) */
    vqf_real_t magMinUndisturbedTime;    /**< Minimum undisturbed time to clear disturbance flag (default: 0.5 s) */
    vqf_real_t magMaxRejectionTime;      /**< Maximum full magnetic rejection duration (default: 60.0 s) */
    vqf_real_t magRejectionFactor;       /**< Factor to slow heading correction during long disturbances (default: 2.0) */
} VQFParams;

/*
 * ----------------------------------------------------------------------------
 *  VQFState — Everything the filter remembers between samples
 * ----------------------------------------------------------------------------
 *  VQF splits the total orientation into two quaternions that are multiplied
 *  together at output time:
 *
 *    gyrQuat  — "Where gyro integration says I am pointing"
 *    accQuat  — "The correction that aligns gyrQuat with gravity"
 *    delta    — Additional yaw correction from the magnetometer
 *
 *  Final 6D output  = accQuat * gyrQuat
 *  Final 9D output  = rotate(6D output, delta)
 *
 *  The remaining fields (bias, Kalman covariance P, low-pass filter states,
 *  magnetometer tracking) are internal bookkeeping.  You do not normally
 *  need to read them; use the getter functions instead.
 * ----------------------------------------------------------------------------
 */
/**
 * @brief Filter state of the VQF algorithm.
 */
typedef struct {
    /* The raw gyro-only orientation: integrated spin, no corrections yet. */
    vqf_real_t gyrQuat[4];               /**< Gyroscope strapdown integration quaternion */

    /* The tilt correction quaternion: rotates gyrQuat so "down" matches accel. */
    vqf_real_t accQuat[4];               /**< Inclination correction quaternion */

    /* Additional yaw (compass heading) offset applied on top of 6D result. */
    vqf_real_t delta;                    /**< Heading difference between Ei and E */
    bool restDetected;                   /**< True if rest is detected */
    bool magDistDetected;                /**< True if magnetic disturbance is detected */

    vqf_real_t lastAccLp[3];             /**< Last low-pass filtered acceleration in Ii frame */
    double accLpState[3*2];              /**< Internal low-pass filter state for lastAccLp */
    vqf_real_t lastAccCorrAngularRate;   /**< Last inclination correction angular rate */

    vqf_real_t kMagInit;                 /**< Initial convergence gain for heading correction */
    vqf_real_t lastMagDisAngle;          /**< Last heading disagreement angle */
    vqf_real_t lastMagCorrAngularRate;   /**< Last heading correction angular rate */

    vqf_real_t bias[3];                  /**< Current gyroscope bias estimate (rad/s) */
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    vqf_real_t biasP[9];                 /**< Covariance matrix of bias estimate (3x3, row-major) */
#else
    vqf_real_t biasP;                    /**< Scalar covariance (rest-only mode) */
#endif

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    double motionBiasEstRLpState[9*2];   /**< Low-pass filter state for rotation matrix in motion bias est */
    double motionBiasEstBiasLpState[2*2]; /**< Low-pass filter state for bias in motion bias est */
#endif

    vqf_real_t restLastSquaredDeviations[2]; /**< Last squared deviations for rest detection */
    vqf_real_t restT;                    /**< Duration within rest thresholds */
    vqf_real_t restLastGyrLp[3];         /**< Last low-pass filtered gyro for rest detection */
    double restGyrLpState[3*2];          /**< Low-pass filter state for rest gyro */
    vqf_real_t restLastAccLp[3];         /**< Last low-pass filtered acc for rest detection */
    double restAccLpState[3*2];          /**< Low-pass filter state for rest acc */

    vqf_real_t magRefNorm;               /**< Norm of accepted magnetic field reference */
    vqf_real_t magRefDip;                /**< Dip angle of accepted magnetic field reference */
    vqf_real_t magUndisturbedT;          /**< Duration close to reference */
    vqf_real_t magRejectT;               /**< Duration of magnetic rejection */
    vqf_real_t magCandidateNorm;         /**< Norm of candidate magnetic field */
    vqf_real_t magCandidateDip;          /**< Dip angle of candidate magnetic field */
    vqf_real_t magCandidateT;            /**< Duration close to candidate */
    vqf_real_t magNormDip[2];            /**< Current mag norm and dip (low-pass filtered) */
    double magNormDipLpState[2*2];       /**< Low-pass filter state for mag norm/dip */
} VQFState;

/**
 * @brief Coefficients computed from parameters and sampling rates.
 */
typedef struct {
    vqf_real_t gyrTs;                    /**< Gyroscope sampling time (s) */
    vqf_real_t accTs;                    /**< Accelerometer sampling time (s) */
    vqf_real_t magTs;                    /**< Magnetometer sampling time (s) */

    double accLpB[3];                    /**< Acceleration low-pass filter numerator coefficients */
    double accLpA[2];                    /**< Acceleration low-pass filter denominator coefficients */

    vqf_real_t kMag;                     /**< Heading correction filter gain */

    vqf_real_t biasP0;                   /**< Initial bias estimation variance */
    vqf_real_t biasV;                    /**< System noise variance for bias estimation */
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    vqf_real_t biasMotionW;              /**< Motion bias estimation measurement noise variance */
    vqf_real_t biasVerticalW;            /**< Vertical motion bias measurement noise variance */
#endif
    vqf_real_t biasRestW;                /**< Rest bias estimation measurement noise variance */

    double restGyrLpB[3];                /**< Rest detection gyro low-pass numerator */
    double restGyrLpA[2];                /**< Rest detection gyro low-pass denominator */
    double restAccLpB[3];                /**< Rest detection acc low-pass numerator */
    double restAccLpA[2];                /**< Rest detection acc low-pass denominator */

    vqf_real_t kMagRef;                  /**< Magnetic field reference update gain */
    double magNormDipLpB[3];             /**< Mag norm/dip low-pass numerator */
    double magNormDipLpA[2];             /**< Mag norm/dip low-pass denominator */
} VQFCoefficients;

/**
 * @brief Main VQF filter instance.
 */
typedef struct {
    VQFParams params;
    VQFState state;
    VQFCoefficients coeffs;
} VQF;

/* ---- Initialization ---- */

/** @brief Fill *params with safe default values (call before vqf_init_params). */
void vqf_params_init(VQFParams *params);

/* Call vqf_init once per sensor before the data loop.
 * gyrTs / accTs / magTs = 1.0 / sample_rate_Hz for each sensor.
 * If the sensor has no magnetometer, pass 0 for magTs. */
/** @brief Initialize VQF with default parameters. */
void vqf_init(VQF *vqf, vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs);

/** @brief Initialize VQF with custom parameters (call vqf_params_init first). */
void vqf_init_params(VQF *vqf, const VQFParams *params, vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs);

/* ---- Update steps (call once per sample in the data loop) ---- */

/* gyr[3] = angular velocity in rad/s, axes x/y/z.
 * This integrates the spin to advance gyrQuat by one time step.
 * Must be called every sample even when accel/mag are unavailable. */
/** @brief Perform gyroscope update step (always required). */
void vqf_update_gyr(VQF *vqf, const vqf_real_t gyr[3]);

/* acc[3] = linear acceleration in m/s², axes x/y/z.
 * Compares the predicted gravity direction to the measured one and
 * gently corrects accQuat.  Also feeds the gyro bias estimator. */
/** @brief Perform accelerometer update step (tilt correction). */
void vqf_update_acc(VQF *vqf, const vqf_real_t acc[3]);

/* mag[3] = magnetic field in any consistent unit (e.g. µT), axes x/y/z.
 * Corrects the yaw angle (delta) toward magnetic north.
 * Automatically skipped when magnetic disturbance is detected. */
/** @brief Perform magnetometer update step (heading correction). */
void vqf_update_mag(VQF *vqf, const vqf_real_t mag[3]);

/* Convenience: run vqf_update_gyr then vqf_update_acc in one call. */
/** @brief Perform 6D update (gyroscope + accelerometer). */
void vqf_update(VQF *vqf, const vqf_real_t gyr[3], const vqf_real_t acc[3]);

/* Convenience: run all three updates in one call. */
/** @brief Perform 9D update (gyroscope + accelerometer + magnetometer). */
void vqf_update_9d(VQF *vqf, const vqf_real_t gyr[3], const vqf_real_t acc[3], const vqf_real_t mag[3]);

/** @brief Perform batch update for multiple samples. */
void vqf_update_batch(VQF *vqf, const vqf_real_t gyr[], const vqf_real_t acc[], const vqf_real_t mag[],
                      size_t N, vqf_real_t out6D[], vqf_real_t out9D[], vqf_real_t outDelta[],
                      vqf_real_t outBias[], vqf_real_t outBiasSigma[], bool outRest[], bool outMagDist[]);

/* ---- Output getters (read after each update) ---- */

/* out[4] = {w, x, y, z}.  All three functions write a unit quaternion.
 * Choose the one that matches your update mode:
 *   3D: gyro only — drifts over time, useful for fast motions only
 *   6D: gyro + accel — stable tilt, yaw drifts slowly (no north reference)
 *   9D: all three — fully stable when mag is reliable              */
/** @brief Get 3D quaternion (gyro strapdown only, no drift correction). */
void vqf_get_quat_3d(const VQF *vqf, vqf_real_t out[4]);

/** @brief Get 6D quaternion (tilt-corrected, yaw drifts slowly). */
void vqf_get_quat_6d(const VQF *vqf, vqf_real_t out[4]);

/** @brief Get 9D quaternion (full orientation, requires magnetometer). */
void vqf_get_quat_9d(const VQF *vqf, vqf_real_t out[4]);

/** @brief Get heading difference delta. */
vqf_real_t vqf_get_delta(const VQF *vqf);

/** @brief Get gyroscope bias estimate and return estimation uncertainty sigma. */
vqf_real_t vqf_get_bias_estimate(const VQF *vqf, vqf_real_t out[3]);

/** @brief Set gyroscope bias estimate. Set sigma to -1 to keep current covariance. */
void vqf_set_bias_estimate(VQF *vqf, vqf_real_t bias[3], vqf_real_t sigma);

/** @brief Returns true if rest was detected. */
bool vqf_get_rest_detected(const VQF *vqf);

/** @brief Returns true if magnetic disturbance was detected. */
bool vqf_get_mag_dist_detected(const VQF *vqf);

/** @brief Get relative rest deviations (out[0]=gyro, out[1]=acc, both relative to threshold). */
void vqf_get_relative_rest_deviations(const VQF *vqf, vqf_real_t out[2]);

/** @brief Get norm of the accepted magnetic field reference. */
vqf_real_t vqf_get_mag_ref_norm(const VQF *vqf);

/** @brief Get dip angle of the accepted magnetic field reference. */
vqf_real_t vqf_get_mag_ref_dip(const VQF *vqf);

/** @brief Set the magnetic field reference. */
void vqf_set_mag_ref(VQF *vqf, vqf_real_t norm, vqf_real_t dip);

/* ---- Parameter setters ---- */

void vqf_set_tau_acc(VQF *vqf, vqf_real_t tauAcc);
void vqf_set_tau_mag(VQF *vqf, vqf_real_t tauMag);
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
void vqf_set_motion_bias_est_enabled(VQF *vqf, bool enabled);
#endif
void vqf_set_rest_bias_est_enabled(VQF *vqf, bool enabled);
void vqf_set_mag_dist_rejection_enabled(VQF *vqf, bool enabled);
void vqf_set_rest_detection_thresholds(VQF *vqf, vqf_real_t thGyr, vqf_real_t thAcc);

/* ---- State access ---- */

const VQFParams *vqf_get_params(const VQF *vqf);
const VQFCoefficients *vqf_get_coeffs(const VQF *vqf);
const VQFState *vqf_get_state(const VQF *vqf);
void vqf_set_state(VQF *vqf, const VQFState *state);
void vqf_reset_state(VQF *vqf);

/* ---- Utility functions (quaternion math, filtering) ---- */

/** @brief Quaternion multiplication: out = q1 * q2. */
void vqf_quat_multiply(const vqf_real_t q1[4], const vqf_real_t q2[4], vqf_real_t out[4]);

/** @brief Quaternion conjugate: out = q*. */
void vqf_quat_conj(const vqf_real_t q[4], vqf_real_t out[4]);

/** @brief Set quaternion to identity [1, 0, 0, 0]. */
void vqf_quat_set_to_identity(vqf_real_t out[4]);

/** @brief Apply heading rotation: out = [cos(delta/2), 0, 0, sin(delta/2)] * q. */
void vqf_quat_apply_delta(vqf_real_t q[4], vqf_real_t delta, vqf_real_t out[4]);

/** @brief Rotate vector by quaternion: out = q * [0,v] * q*. */
void vqf_quat_rotate(const vqf_real_t q[4], const vqf_real_t v[3], vqf_real_t out[3]);

/** @brief Euclidean norm of a vector. */
vqf_real_t vqf_norm(const vqf_real_t vec[], size_t N);

/** @brief Normalize a vector in-place. */
void vqf_normalize(vqf_real_t vec[], size_t N);

/** @brief Clip vector elements to [min, max] in-place. */
void vqf_clip(vqf_real_t vec[], size_t N, vqf_real_t min, vqf_real_t max);

/** @brief Compute first-order low-pass filter gain from time constant. */
vqf_real_t vqf_gain_from_tau(vqf_real_t tau, vqf_real_t Ts);

/** @brief Compute second-order Butterworth low-pass filter coefficients. */
void vqf_filter_coeffs(vqf_real_t tau, vqf_real_t Ts, double outB[3], double outA[2]);

/** @brief Compute initial filter state for a steady-state value. */
void vqf_filter_initial_state(vqf_real_t x0, const double b[3], const double a[2], double out[2]);

/** @brief Adapt filter state when changing coefficients. */
void vqf_filter_adapt_state_for_coeff_change(vqf_real_t last_y[], size_t N, const double b_old[3],
                                              const double a_old[2], const double b_new[3],
                                              const double a_new[2], double state[]);

/** @brief Single filter step for a scalar value. */
vqf_real_t vqf_filter_step(vqf_real_t x, const double b[3], const double a[2], double state[2]);

/** @brief Filter step for vector-valued signal with averaging-based initialization. */
void vqf_filter_vec(const vqf_real_t x[], size_t N, vqf_real_t tau, vqf_real_t Ts,
                    const double b[3], const double a[2], double state[], vqf_real_t out[]);

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
/** @brief Set 3x3 matrix to scaled identity. */
void vqf_matrix3_set_to_scaled_identity(vqf_real_t scale, vqf_real_t out[9]);

/** @brief 3x3 matrix multiplication: out = in1 * in2. */
void vqf_matrix3_multiply(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9]);

/** @brief 3x3 matrix multiply with first matrix transposed: out = in1^T * in2. */
void vqf_matrix3_multiply_tps_first(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9]);

/** @brief 3x3 matrix multiply with second matrix transposed: out = in1 * in2^T. */
void vqf_matrix3_multiply_tps_second(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9]);

/** @brief 3x3 matrix inverse: out = in^-1. Returns false if singular. */
bool vqf_matrix3_inv(const vqf_real_t in[9], vqf_real_t out[9]);
#endif

#ifdef __cplusplus
}
#endif

#endif /* VQF_H */
