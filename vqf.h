// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
//
// SPDX-License-Identifier: MIT
//
// Pure C conversion for Red Pitaya embedded use.
// Original C++ implementation by Daniel Laidig.

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

/**
 * @brief Tuning parameters for the VQF algorithm.
 */
typedef struct {
    vqf_real_t tauAcc;                   /**< Accelerometer low-pass filter time constant (default: 3.0 s) */
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

/**
 * @brief Filter state of the VQF algorithm.
 */
typedef struct {
    vqf_real_t gyrQuat[4];               /**< Gyroscope strapdown integration quaternion */
    vqf_real_t accQuat[4];               /**< Inclination correction quaternion */
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

/** @brief Initialize VQFParams with default values. */
void vqf_params_init(VQFParams *params);

/** @brief Initialize VQF with default parameters. */
void vqf_init(VQF *vqf, vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs);

/** @brief Initialize VQF with custom parameters. */
void vqf_init_params(VQF *vqf, const VQFParams *params, vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs);

/* ---- Update steps ---- */

/** @brief Perform gyroscope update step. */
void vqf_update_gyr(VQF *vqf, const vqf_real_t gyr[3]);

/** @brief Perform accelerometer update step. */
void vqf_update_acc(VQF *vqf, const vqf_real_t acc[3]);

/** @brief Perform magnetometer update step. */
void vqf_update_mag(VQF *vqf, const vqf_real_t mag[3]);

/** @brief Perform 6D update (gyroscope + accelerometer). */
void vqf_update(VQF *vqf, const vqf_real_t gyr[3], const vqf_real_t acc[3]);

/** @brief Perform 9D update (gyroscope + accelerometer + magnetometer). */
void vqf_update_9d(VQF *vqf, const vqf_real_t gyr[3], const vqf_real_t acc[3], const vqf_real_t mag[3]);

/** @brief Perform batch update for multiple samples. */
void vqf_update_batch(VQF *vqf, const vqf_real_t gyr[], const vqf_real_t acc[], const vqf_real_t mag[],
                      size_t N, vqf_real_t out6D[], vqf_real_t out9D[], vqf_real_t outDelta[],
                      vqf_real_t outBias[], vqf_real_t outBiasSigma[], bool outRest[], bool outMagDist[]);

/* ---- Output getters ---- */

/** @brief Get 3D quaternion (gyro strapdown only). */
void vqf_get_quat_3d(const VQF *vqf, vqf_real_t out[4]);

/** @brief Get 6D quaternion (magnetometer-free). */
void vqf_get_quat_6d(const VQF *vqf, vqf_real_t out[4]);

/** @brief Get 9D quaternion (with magnetometer). */
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
