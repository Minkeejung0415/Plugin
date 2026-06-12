// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
//
// SPDX-License-Identifier: MIT
//
// Pure C conversion for Red Pitaya embedded use.

#include "vqf.h"

#include <math.h>
#include <float.h>
#include <string.h>
#if !defined(ARDUINO) && !defined(ESP_PLATFORM)
#include <assert.h>
#endif

#if defined(ARDUINO) || defined(ESP_PLATFORM) || defined(NDEBUG)
#define VQF_ASSERT(cond) ((void)0)
#else
#define VQF_ASSERT(cond) assert(cond)
#endif

#ifndef VQF_SINGLE_PRECISION
#define VQF_EPS DBL_EPSILON
#else
#define VQF_EPS FLT_EPSILON
#endif

#define VQF_NAN_VAL   ((double)NAN)
#define VQF_PI        3.14159265358979323846264338327950288
#define VQF_SQRT2     1.41421356237309504880168872420969808
#define VQF_MAX(a, b) ((a) > (b) ? (a) : (b))
#define VQF_MIN(a, b) ((a) < (b) ? (a) : (b))

static inline vqf_real_t square(vqf_real_t x) { return x * x; }

static void fill_real(vqf_real_t *a, size_t n, vqf_real_t v)
{
    for (size_t i = 0; i < n; i++) a[i] = v;
}

static void fill_double(double *a, size_t n, double v)
{
    for (size_t i = 0; i < n; i++) a[i] = v;
}

/* ================================================================ */
/*  VQFParams default init                                          */
/* ================================================================ */

void vqf_params_init(VQFParams *p)
{
    p->tauAcc = 3.0;
    p->tauMag = 9.0;
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    p->motionBiasEstEnabled = true;
#endif
    p->restBiasEstEnabled = true;
    p->magDistRejectionEnabled = true;
    p->biasSigmaInit = 0.5;
    p->biasForgettingTime = 100.0;
    p->biasClip = 2.0;
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    p->biasSigmaMotion = 0.1;
    p->biasVerticalForgettingFactor = 0.0001;
#endif
    p->biasSigmaRest = 0.03;
    p->restMinT = 1.5;
    p->restFilterTau = 0.5;
    p->restThGyr = 2.0;
    p->restThAcc = 0.5;
    p->magCurrentTau = 0.05;
    p->magRefTau = 20.0;
    p->magNormTh = 0.1;
    p->magDipTh = 10.0;
    p->magNewTime = 20.0;
    p->magNewFirstTime = 5.0;
    p->magNewMinGyr = 20.0;
    p->magMinUndisturbedTime = 0.5;
    p->magMaxRejectionTime = 60.0;
    p->magRejectionFactor = 2.0;
}

/* ================================================================ */
/*  Quaternion / vector / filter utilities                          */
/* ================================================================ */

void vqf_quat_multiply(const vqf_real_t q1[4], const vqf_real_t q2[4], vqf_real_t out[4])
{
    vqf_real_t w = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    vqf_real_t x = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    vqf_real_t y = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    vqf_real_t z = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

void vqf_quat_conj(const vqf_real_t q[4], vqf_real_t out[4])
{
    out[0] = q[0]; out[1] = -q[1]; out[2] = -q[2]; out[3] = -q[3];
}

void vqf_quat_set_to_identity(vqf_real_t out[4])
{
    out[0] = 1; out[1] = 0; out[2] = 0; out[3] = 0;
}

void vqf_quat_apply_delta(vqf_real_t q[4], vqf_real_t delta, vqf_real_t out[4])
{
    vqf_real_t c = cos(delta / 2);
    vqf_real_t s = sin(delta / 2);
    vqf_real_t w = c*q[0] - s*q[3];
    vqf_real_t x = c*q[1] - s*q[2];
    vqf_real_t y = c*q[2] + s*q[1];
    vqf_real_t z = c*q[3] + s*q[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

void vqf_quat_rotate(const vqf_real_t q[4], const vqf_real_t v[3], vqf_real_t out[3])
{
    vqf_real_t x = (1 - 2*q[2]*q[2] - 2*q[3]*q[3])*v[0]
                 + 2*v[1]*(q[2]*q[1] - q[0]*q[3])
                 + 2*v[2]*(q[0]*q[2] + q[3]*q[1]);
    vqf_real_t y = 2*v[0]*(q[0]*q[3] + q[2]*q[1])
                 + v[1]*(1 - 2*q[1]*q[1] - 2*q[3]*q[3])
                 + 2*v[2]*(q[2]*q[3] - q[1]*q[0]);
    vqf_real_t z = 2*v[0]*(q[3]*q[1] - q[0]*q[2])
                 + 2*v[1]*(q[0]*q[1] + q[3]*q[2])
                 + v[2]*(1 - 2*q[1]*q[1] - 2*q[2]*q[2]);
    out[0] = x; out[1] = y; out[2] = z;
}

vqf_real_t vqf_norm(const vqf_real_t vec[], size_t N)
{
    vqf_real_t s = 0;
    for (size_t i = 0; i < N; i++) s += vec[i]*vec[i];
    return sqrt(s);
}

void vqf_normalize(vqf_real_t vec[], size_t N)
{
    vqf_real_t n = vqf_norm(vec, N);
    if (n < VQF_EPS) return;
    for (size_t i = 0; i < N; i++) vec[i] /= n;
}

void vqf_clip(vqf_real_t vec[], size_t N, vqf_real_t mn, vqf_real_t mx)
{
    for (size_t i = 0; i < N; i++) {
        if (vec[i] < mn) vec[i] = mn;
        else if (vec[i] > mx) vec[i] = mx;
    }
}

vqf_real_t vqf_gain_from_tau(vqf_real_t tau, vqf_real_t Ts)
{
    VQF_ASSERT(Ts > 0);
    if (tau < 0) return 0;
    if (tau == 0) return 1;
    return 1 - exp(-Ts / tau);
}

void vqf_filter_coeffs(vqf_real_t tau, vqf_real_t Ts, double outB[3], double outA[2])
{
    VQF_ASSERT(tau > 0);
    VQF_ASSERT(Ts > 0);
    if (tau < Ts / 2) {
        outB[0] = 1; outB[1] = 0; outB[2] = 0;
        outA[0] = 0; outA[1] = 0;
        return;
    }
    double fc = (VQF_SQRT2 / (2.0 * VQF_PI)) / (double)tau;
    double C  = tan(VQF_PI * fc * (double)Ts);
    double D  = C*C + sqrt(2.0)*C + 1;
    double b0 = C*C / D;
    outB[0] = b0;
    outB[1] = 2*b0;
    outB[2] = b0;
    outA[0] = 2*(C*C - 1) / D;
    outA[1] = (1 - sqrt(2.0)*C + C*C) / D;
}

void vqf_filter_initial_state(vqf_real_t x0, const double b[3], const double a[2], double out[2])
{
    out[0] = (double)x0 * (1 - b[0]);
    out[1] = (double)x0 * (b[2] - a[1]);
}

void vqf_filter_adapt_state_for_coeff_change(vqf_real_t last_y[], size_t N,
    const double b_old[3], const double a_old[2],
    const double b_new[3], const double a_new[2], double state[])
{
    if (isnan(state[0])) return;
    for (size_t i = 0; i < N; i++) {
        state[0+2*i] += (b_old[0] - b_new[0]) * (double)last_y[i];
        state[1+2*i] += (b_old[1] - b_new[1] - a_old[0] + a_new[0]) * (double)last_y[i];
    }
}

vqf_real_t vqf_filter_step(vqf_real_t x, const double b[3], const double a[2], double state[2])
{
    double y   = b[0]*(double)x + state[0];
    state[0]   = b[1]*(double)x - a[0]*y + state[1];
    state[1]   = b[2]*(double)x - a[1]*y;
    return (vqf_real_t)y;
}

void vqf_filter_vec(const vqf_real_t x[], size_t N, vqf_real_t tau, vqf_real_t Ts,
                    const double b[3], const double a[2], double state[], vqf_real_t out[])
{
    VQF_ASSERT(N >= 2);
    if (isnan(state[0])) {                     /* initialization phase */
        if (isnan(state[1])) {                 /* first sample */
            state[1] = 0;
            for (size_t i = 0; i < N; i++) state[2+i] = 0;
        }
        state[1]++;
        for (size_t i = 0; i < N; i++) {
            state[2+i] += (double)x[i];
            out[i] = (vqf_real_t)(state[2+i] / state[1]);
        }
        if ((vqf_real_t)state[1] * Ts >= tau) {
            for (size_t i = 0; i < N; i++)
                vqf_filter_initial_state(out[i], b, a, state + 2*i);
        }
        return;
    }
    for (size_t i = 0; i < N; i++)
        out[i] = vqf_filter_step(x[i], b, a, state + 2*i);
}

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
void vqf_matrix3_set_to_scaled_identity(vqf_real_t scale, vqf_real_t out[9])
{
    out[0]=scale; out[1]=0; out[2]=0;
    out[3]=0; out[4]=scale; out[5]=0;
    out[6]=0; out[7]=0; out[8]=scale;
}

void vqf_matrix3_multiply(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9])
{
    vqf_real_t t[9];
    t[0]=in1[0]*in2[0]+in1[1]*in2[3]+in1[2]*in2[6];
    t[1]=in1[0]*in2[1]+in1[1]*in2[4]+in1[2]*in2[7];
    t[2]=in1[0]*in2[2]+in1[1]*in2[5]+in1[2]*in2[8];
    t[3]=in1[3]*in2[0]+in1[4]*in2[3]+in1[5]*in2[6];
    t[4]=in1[3]*in2[1]+in1[4]*in2[4]+in1[5]*in2[7];
    t[5]=in1[3]*in2[2]+in1[4]*in2[5]+in1[5]*in2[8];
    t[6]=in1[6]*in2[0]+in1[7]*in2[3]+in1[8]*in2[6];
    t[7]=in1[6]*in2[1]+in1[7]*in2[4]+in1[8]*in2[7];
    t[8]=in1[6]*in2[2]+in1[7]*in2[5]+in1[8]*in2[8];
    memcpy(out, t, sizeof t);
}

void vqf_matrix3_multiply_tps_first(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9])
{
    vqf_real_t t[9];
    t[0]=in1[0]*in2[0]+in1[3]*in2[3]+in1[6]*in2[6];
    t[1]=in1[0]*in2[1]+in1[3]*in2[4]+in1[6]*in2[7];
    t[2]=in1[0]*in2[2]+in1[3]*in2[5]+in1[6]*in2[8];
    t[3]=in1[1]*in2[0]+in1[4]*in2[3]+in1[7]*in2[6];
    t[4]=in1[1]*in2[1]+in1[4]*in2[4]+in1[7]*in2[7];
    t[5]=in1[1]*in2[2]+in1[4]*in2[5]+in1[7]*in2[8];
    t[6]=in1[2]*in2[0]+in1[5]*in2[3]+in1[8]*in2[6];
    t[7]=in1[2]*in2[1]+in1[5]*in2[4]+in1[8]*in2[7];
    t[8]=in1[2]*in2[2]+in1[5]*in2[5]+in1[8]*in2[8];
    memcpy(out, t, sizeof t);
}

void vqf_matrix3_multiply_tps_second(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9])
{
    vqf_real_t t[9];
    t[0]=in1[0]*in2[0]+in1[1]*in2[1]+in1[2]*in2[2];
    t[1]=in1[0]*in2[3]+in1[1]*in2[4]+in1[2]*in2[5];
    t[2]=in1[0]*in2[6]+in1[1]*in2[7]+in1[2]*in2[8];
    t[3]=in1[3]*in2[0]+in1[4]*in2[1]+in1[5]*in2[2];
    t[4]=in1[3]*in2[3]+in1[4]*in2[4]+in1[5]*in2[5];
    t[5]=in1[3]*in2[6]+in1[4]*in2[7]+in1[5]*in2[8];
    t[6]=in1[6]*in2[0]+in1[7]*in2[1]+in1[8]*in2[2];
    t[7]=in1[6]*in2[3]+in1[7]*in2[4]+in1[8]*in2[5];
    t[8]=in1[6]*in2[6]+in1[7]*in2[7]+in1[8]*in2[8];
    memcpy(out, t, sizeof t);
}

bool vqf_matrix3_inv(const vqf_real_t in[9], vqf_real_t out[9])
{
    double A = (double)(in[4]*in[8] - in[5]*in[7]);
    double D = (double)(in[2]*in[7] - in[1]*in[8]);
    double G = (double)(in[1]*in[5] - in[2]*in[4]);
    double B = (double)(in[5]*in[6] - in[3]*in[8]);
    double E = (double)(in[0]*in[8] - in[2]*in[6]);
    double H = (double)(in[2]*in[3] - in[0]*in[5]);
    double CC= (double)(in[3]*in[7] - in[4]*in[6]);
    double F = (double)(in[1]*in[6] - in[0]*in[7]);
    double I = (double)(in[0]*in[4] - in[1]*in[3]);
    double det = (double)in[0]*A + (double)in[1]*B + (double)in[2]*CC;
    if (det >= -VQF_EPS && det <= VQF_EPS) {
        memset(out, 0, 9*sizeof(vqf_real_t));
        return false;
    }
    out[0]=(vqf_real_t)(A/det); out[1]=(vqf_real_t)(D/det); out[2]=(vqf_real_t)(G/det);
    out[3]=(vqf_real_t)(B/det); out[4]=(vqf_real_t)(E/det); out[5]=(vqf_real_t)(H/det);
    out[6]=(vqf_real_t)(CC/det);out[7]=(vqf_real_t)(F/det); out[8]=(vqf_real_t)(I/det);
    return true;
}
#endif /* VQF_NO_MOTION_BIAS_ESTIMATION */

/* ================================================================ */
/*  Internal setup                                                  */
/* ================================================================ */

static void vqf_setup(VQF *vqf)
{
    VQFParams *p = &vqf->params;
    VQFCoefficients *c = &vqf->coeffs;

    VQF_ASSERT(c->gyrTs > 0);
    VQF_ASSERT(c->accTs > 0);
    VQF_ASSERT(c->magTs > 0);

    vqf_filter_coeffs(p->tauAcc, c->accTs, c->accLpB, c->accLpA);
    c->kMag = vqf_gain_from_tau(p->tauMag, c->magTs);
    c->biasP0 = square(p->biasSigmaInit * (vqf_real_t)100.0);
    c->biasV  = square(0.1 * 100.0) * c->accTs / p->biasForgettingTime;

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    {
        vqf_real_t pM = square(p->biasSigmaMotion * (vqf_real_t)100.0);
        c->biasMotionW   = square(pM) / c->biasV + pM;
        c->biasVerticalW = c->biasMotionW / VQF_MAX(p->biasVerticalForgettingFactor, (vqf_real_t)1e-10);
    }
#endif
    {
        vqf_real_t pR = square(p->biasSigmaRest * (vqf_real_t)100.0);
        c->biasRestW = square(pR) / c->biasV + pR;
    }

    vqf_filter_coeffs(p->restFilterTau, c->gyrTs, c->restGyrLpB, c->restGyrLpA);
    vqf_filter_coeffs(p->restFilterTau, c->accTs, c->restAccLpB, c->restAccLpA);

    c->kMagRef = vqf_gain_from_tau(p->magRefTau, c->magTs);
    if (p->magCurrentTau > 0)
        vqf_filter_coeffs(p->magCurrentTau, c->magTs, c->magNormDipLpB, c->magNormDipLpA);
    else {
        fill_double(c->magNormDipLpB, 3, VQF_NAN_VAL);
        fill_double(c->magNormDipLpA, 2, VQF_NAN_VAL);
    }

    vqf_reset_state(vqf);
}

/* ================================================================ */
/*  Init                                                            */
/* ================================================================ */

void vqf_init(VQF *vqf, vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs)
{
    vqf_params_init(&vqf->params);
    vqf->coeffs.gyrTs = gyrTs;
    vqf->coeffs.accTs = accTs > 0 ? accTs : gyrTs;
    vqf->coeffs.magTs = magTs > 0 ? magTs : gyrTs;
    vqf_setup(vqf);
}

void vqf_init_params(VQF *vqf, const VQFParams *params, vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs)
{
    vqf->params = *params;
    vqf->coeffs.gyrTs = gyrTs;
    vqf->coeffs.accTs = accTs > 0 ? accTs : gyrTs;
    vqf->coeffs.magTs = magTs > 0 ? magTs : gyrTs;
    vqf_setup(vqf);
}

/* ================================================================ */
/*  Update steps                                                    */
/* ================================================================ */

void vqf_update_gyr(VQF *vqf, const vqf_real_t gyr[3])
{
    VQFParams *p = &vqf->params;
    VQFState  *s = &vqf->state;
    VQFCoefficients *c = &vqf->coeffs;

    if (p->restBiasEstEnabled || p->magDistRejectionEnabled) {
        vqf_filter_vec(gyr, 3, p->restFilterTau, c->gyrTs,
                       c->restGyrLpB, c->restGyrLpA, s->restGyrLpState, s->restLastGyrLp);
        s->restLastSquaredDeviations[0] =
            square(gyr[0]-s->restLastGyrLp[0]) +
            square(gyr[1]-s->restLastGyrLp[1]) +
            square(gyr[2]-s->restLastGyrLp[2]);
        vqf_real_t bc = p->biasClip*(vqf_real_t)(VQF_PI/180.0);
        if (s->restLastSquaredDeviations[0] >= square(p->restThGyr*(vqf_real_t)(VQF_PI/180.0))
                || fabs(s->restLastGyrLp[0]) > bc
                || fabs(s->restLastGyrLp[1]) > bc
                || fabs(s->restLastGyrLp[2]) > bc) {
            s->restT = 0.0;
            s->restDetected = false;
        }
    }

    vqf_real_t gyrNoBias[3] = {gyr[0]-s->bias[0], gyr[1]-s->bias[1], gyr[2]-s->bias[2]};
    vqf_real_t gn = vqf_norm(gyrNoBias, 3);
    vqf_real_t angle = gn * c->gyrTs;
    if (gn > VQF_EPS) {
        vqf_real_t co = cos(angle/2);
        vqf_real_t si = sin(angle/2)/gn;
        vqf_real_t gq[4] = {co, si*gyrNoBias[0], si*gyrNoBias[1], si*gyrNoBias[2]};
        vqf_quat_multiply(s->gyrQuat, gq, s->gyrQuat);
        vqf_normalize(s->gyrQuat, 4);
    }
}

void vqf_update_acc(VQF *vqf, const vqf_real_t acc[3])
{
    VQFParams *p = &vqf->params;
    VQFState  *s = &vqf->state;
    VQFCoefficients *c = &vqf->coeffs;

    if (acc[0]==0 && acc[1]==0 && acc[2]==0) return;

    if (p->restBiasEstEnabled) {
        vqf_filter_vec(acc, 3, p->restFilterTau, c->accTs,
                       c->restAccLpB, c->restAccLpA, s->restAccLpState, s->restLastAccLp);
        s->restLastSquaredDeviations[1] =
            square(acc[0]-s->restLastAccLp[0]) +
            square(acc[1]-s->restLastAccLp[1]) +
            square(acc[2]-s->restLastAccLp[2]);
        if (s->restLastSquaredDeviations[1] >= square(p->restThAcc)) {
            s->restT = 0.0;
            s->restDetected = false;
        } else {
            s->restT += c->accTs;
            if (s->restT >= p->restMinT) s->restDetected = true;
        }
    }

    vqf_real_t accE[3];
    vqf_quat_rotate(s->gyrQuat, acc, accE);
    vqf_filter_vec(accE, 3, p->tauAcc, c->accTs, c->accLpB, c->accLpA, s->accLpState, s->lastAccLp);
    vqf_quat_rotate(s->accQuat, s->lastAccLp, accE);
    vqf_normalize(accE, 3);

    vqf_real_t accCQ[4];
    vqf_real_t q_w = sqrt((accE[2]+1)/2);
    if (q_w > (vqf_real_t)1e-6) {
        accCQ[0] = q_w;
        accCQ[1] = (vqf_real_t)0.5*accE[1]/q_w;
        accCQ[2] = (vqf_real_t)(-0.5)*accE[0]/q_w;
        accCQ[3] = 0;
    } else {
        accCQ[0]=0; accCQ[1]=1; accCQ[2]=0; accCQ[3]=0;
    }
    vqf_quat_multiply(accCQ, s->accQuat, s->accQuat);
    vqf_normalize(s->accQuat, 4);
    s->lastAccCorrAngularRate = acos(accE[2]) / c->accTs;

    /* ---- bias estimation ---- */
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    if (p->motionBiasEstEnabled || p->restBiasEstEnabled) {
        vqf_real_t bc = p->biasClip*(vqf_real_t)(VQF_PI/180.0);
        vqf_real_t q6[4], R[9], bLp[2];
        vqf_get_quat_6d(vqf, q6);
        R[0]=1-2*square(q6[2])-2*square(q6[3]);
        R[1]=2*(q6[2]*q6[1]-q6[0]*q6[3]);
        R[2]=2*(q6[0]*q6[2]+q6[3]*q6[1]);
        R[3]=2*(q6[0]*q6[3]+q6[2]*q6[1]);
        R[4]=1-2*square(q6[1])-2*square(q6[3]);
        R[5]=2*(q6[2]*q6[3]-q6[1]*q6[0]);
        R[6]=2*(q6[3]*q6[1]-q6[0]*q6[2]);
        R[7]=2*(q6[0]*q6[1]+q6[3]*q6[2]);
        R[8]=1-2*square(q6[1])-2*square(q6[2]);

        bLp[0] = R[0]*s->bias[0]+R[1]*s->bias[1]+R[2]*s->bias[2];
        bLp[1] = R[3]*s->bias[0]+R[4]*s->bias[1]+R[5]*s->bias[2];

        vqf_filter_vec(R,    9, p->tauAcc, c->accTs, c->accLpB, c->accLpA, s->motionBiasEstRLpState,    R);
        vqf_filter_vec(bLp,  2, p->tauAcc, c->accTs, c->accLpB, c->accLpA, s->motionBiasEstBiasLpState, bLp);

        vqf_real_t w[3], e[3];
        if (s->restDetected && p->restBiasEstEnabled) {
            e[0]=s->restLastGyrLp[0]-s->bias[0];
            e[1]=s->restLastGyrLp[1]-s->bias[1];
            e[2]=s->restLastGyrLp[2]-s->bias[2];
            vqf_matrix3_set_to_scaled_identity(1.0, R);
            w[0]=c->biasRestW; w[1]=c->biasRestW; w[2]=c->biasRestW;
        } else if (p->motionBiasEstEnabled) {
            e[0] = -accE[1]/c->accTs + bLp[0] - R[0]*s->bias[0]-R[1]*s->bias[1]-R[2]*s->bias[2];
            e[1] =  accE[0]/c->accTs + bLp[1] - R[3]*s->bias[0]-R[4]*s->bias[1]-R[5]*s->bias[2];
            e[2] = -R[6]*s->bias[0]-R[7]*s->bias[1]-R[8]*s->bias[2];
            w[0]=c->biasMotionW; w[1]=c->biasMotionW; w[2]=c->biasVerticalW;
        } else {
            w[0]=-1; w[1]=-1; w[2]=-1;
        }

        if (s->biasP[0] < c->biasP0) s->biasP[0] += c->biasV;
        if (s->biasP[4] < c->biasP0) s->biasP[4] += c->biasV;
        if (s->biasP[8] < c->biasP0) s->biasP[8] += c->biasV;

        if (w[0] >= 0) {
            vqf_clip(e, 3, -bc, bc);
            vqf_real_t K[9];
            vqf_matrix3_multiply_tps_second(s->biasP, R, K);
            vqf_matrix3_multiply(R, K, K);
            K[0]+=w[0]; K[4]+=w[1]; K[8]+=w[2];
            vqf_matrix3_inv(K, K);
            vqf_matrix3_multiply_tps_first(R, K, K);
            vqf_matrix3_multiply(s->biasP, K, K);
            s->bias[0] += K[0]*e[0]+K[1]*e[1]+K[2]*e[2];
            s->bias[1] += K[3]*e[0]+K[4]*e[1]+K[5]*e[2];
            s->bias[2] += K[6]*e[0]+K[7]*e[1]+K[8]*e[2];
            vqf_matrix3_multiply(K, R, K);
            vqf_matrix3_multiply(K, s->biasP, K);
            for (size_t i = 0; i < 9; i++) s->biasP[i] -= K[i];
            vqf_clip(s->bias, 3, -bc, bc);
        }
    }
#else
    if (p->restBiasEstEnabled) {
        vqf_real_t bc = p->biasClip*(vqf_real_t)(VQF_PI/180.0);
        if (s->biasP < c->biasP0) s->biasP += c->biasV;
        if (s->restDetected) {
            vqf_real_t e[3];
            e[0]=s->restLastGyrLp[0]-s->bias[0];
            e[1]=s->restLastGyrLp[1]-s->bias[1];
            e[2]=s->restLastGyrLp[2]-s->bias[2];
            vqf_clip(e, 3, -bc, bc);
            vqf_real_t k = s->biasP / (c->biasRestW + s->biasP);
            s->bias[0] += k*e[0];
            s->bias[1] += k*e[1];
            s->bias[2] += k*e[2];
            s->biasP -= k*s->biasP;
            vqf_clip(s->bias, 3, -bc, bc);
        }
    }
#endif
}

void vqf_update_mag(VQF *vqf, const vqf_real_t mag[3])
{
    VQFParams *p = &vqf->params;
    VQFState  *s = &vqf->state;
    VQFCoefficients *c = &vqf->coeffs;

    if (mag[0]==0 && mag[1]==0 && mag[2]==0) return;

    vqf_real_t mE[3], q6[4];
    vqf_get_quat_6d(vqf, q6);
    vqf_quat_rotate(q6, mag, mE);

    if (p->magDistRejectionEnabled) {
        s->magNormDip[0] = vqf_norm(mE, 3);
        s->magNormDip[1] = -asin(mE[2] / s->magNormDip[0]);

        if (p->magCurrentTau > 0)
            vqf_filter_vec(s->magNormDip, 2, p->magCurrentTau, c->magTs,
                           c->magNormDipLpB, c->magNormDipLpA, s->magNormDipLpState, s->magNormDip);

        if (fabs(s->magNormDip[0]-s->magRefNorm) < p->magNormTh*s->magRefNorm
         && fabs(s->magNormDip[1]-s->magRefDip)  < p->magDipTh*(vqf_real_t)(VQF_PI/180.0)) {
            s->magUndisturbedT += c->magTs;
            if (s->magUndisturbedT >= p->magMinUndisturbedTime) {
                s->magDistDetected = false;
                s->magRefNorm += c->kMagRef*(s->magNormDip[0]-s->magRefNorm);
                s->magRefDip  += c->kMagRef*(s->magNormDip[1]-s->magRefDip);
            }
        } else {
            s->magUndisturbedT = 0.0;
            s->magDistDetected = true;
        }

        if (fabs(s->magNormDip[0]-s->magCandidateNorm) < p->magNormTh*s->magCandidateNorm
         && fabs(s->magNormDip[1]-s->magCandidateDip)  < p->magDipTh*(vqf_real_t)(VQF_PI/180.0)) {
            if (vqf_norm(s->restLastGyrLp,3) >= p->magNewMinGyr*(vqf_real_t)(VQF_PI/180.0))
                s->magCandidateT += c->magTs;
            s->magCandidateNorm += c->kMagRef*(s->magNormDip[0]-s->magCandidateNorm);
            s->magCandidateDip  += c->kMagRef*(s->magNormDip[1]-s->magCandidateDip);
            if (s->magDistDetected && (s->magCandidateT >= p->magNewTime
                || (s->magRefNorm==0 && s->magCandidateT >= p->magNewFirstTime))) {
                s->magRefNorm = s->magCandidateNorm;
                s->magRefDip  = s->magCandidateDip;
                s->magDistDetected = false;
                s->magUndisturbedT = p->magMinUndisturbedTime;
            }
        } else {
            s->magCandidateT    = 0.0;
            s->magCandidateNorm = s->magNormDip[0];
            s->magCandidateDip  = s->magNormDip[1];
        }
    }

    s->lastMagDisAngle = atan2(mE[0], mE[1]) - s->delta;
    if (s->lastMagDisAngle > (vqf_real_t)VQF_PI)        s->lastMagDisAngle -= (vqf_real_t)(2*VQF_PI);
    else if (s->lastMagDisAngle < (vqf_real_t)(-VQF_PI)) s->lastMagDisAngle += (vqf_real_t)(2*VQF_PI);

    vqf_real_t k = c->kMag;

    if (p->magDistRejectionEnabled) {
        if (s->magDistDetected) {
            if (s->magRejectT <= p->magMaxRejectionTime) { s->magRejectT += c->magTs; k = 0; }
            else k /= p->magRejectionFactor;
        } else {
            s->magRejectT = VQF_MAX(s->magRejectT - p->magRejectionFactor*c->magTs, (vqf_real_t)0.0);
        }
    }

    if (s->kMagInit != 0) {
        if (k < s->kMagInit) k = s->kMagInit;
        s->kMagInit = s->kMagInit / (s->kMagInit + 1);
        if (s->kMagInit * p->tauMag < c->magTs) s->kMagInit = 0.0;
    }

    s->delta += k * s->lastMagDisAngle;
    s->lastMagCorrAngularRate = k * s->lastMagDisAngle / c->magTs;
    if (s->delta > (vqf_real_t)VQF_PI)        s->delta -= (vqf_real_t)(2*VQF_PI);
    else if (s->delta < (vqf_real_t)(-VQF_PI)) s->delta += (vqf_real_t)(2*VQF_PI);
}

void vqf_update(VQF *vqf, const vqf_real_t gyr[3], const vqf_real_t acc[3])
{
    vqf_update_gyr(vqf, gyr);
    vqf_update_acc(vqf, acc);
}

void vqf_update_9d(VQF *vqf, const vqf_real_t gyr[3], const vqf_real_t acc[3], const vqf_real_t mag[3])
{
    vqf_update_gyr(vqf, gyr);
    vqf_update_acc(vqf, acc);
    vqf_update_mag(vqf, mag);
}

void vqf_update_batch(VQF *vqf, const vqf_real_t gyr[], const vqf_real_t acc[], const vqf_real_t mag[],
                      size_t N, vqf_real_t out6D[], vqf_real_t out9D[], vqf_real_t outDelta[],
                      vqf_real_t outBias[], vqf_real_t outBiasSigma[], bool outRest[], bool outMagDist[])
{
    for (size_t i = 0; i < N; i++) {
        if (mag) vqf_update_9d(vqf, gyr+3*i, acc+3*i, mag+3*i);
        else     vqf_update(vqf, gyr+3*i, acc+3*i);
        if (out6D)       vqf_get_quat_6d(vqf, out6D+4*i);
        if (out9D)       vqf_get_quat_9d(vqf, out9D+4*i);
        if (outDelta)    outDelta[i] = vqf->state.delta;
        if (outBias)     memcpy(outBias+3*i, vqf->state.bias, 3*sizeof(vqf_real_t));
        if (outBiasSigma)outBiasSigma[i] = vqf_get_bias_estimate(vqf, NULL);
        if (outRest)     outRest[i]    = vqf->state.restDetected;
        if (outMagDist)  outMagDist[i] = vqf->state.magDistDetected;
    }
}

/* ================================================================ */
/*  Getters / setters                                               */
/* ================================================================ */

void vqf_get_quat_3d(const VQF *vqf, vqf_real_t out[4])
{
    memcpy(out, vqf->state.gyrQuat, 4*sizeof(vqf_real_t));
}

void vqf_get_quat_6d(const VQF *vqf, vqf_real_t out[4])
{
    vqf_quat_multiply(vqf->state.accQuat, vqf->state.gyrQuat, out);
}

void vqf_get_quat_9d(const VQF *vqf, vqf_real_t out[4])
{
    vqf_quat_multiply(vqf->state.accQuat, vqf->state.gyrQuat, out);
    vqf_quat_apply_delta(out, vqf->state.delta, out);
}

vqf_real_t vqf_get_delta(const VQF *vqf) { return vqf->state.delta; }

vqf_real_t vqf_get_bias_estimate(const VQF *vqf, vqf_real_t out[3])
{
    if (out) memcpy(out, vqf->state.bias, 3*sizeof(vqf_real_t));
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    vqf_real_t s1 = fabs(vqf->state.biasP[0])+fabs(vqf->state.biasP[1])+fabs(vqf->state.biasP[2]);
    vqf_real_t s2 = fabs(vqf->state.biasP[3])+fabs(vqf->state.biasP[4])+fabs(vqf->state.biasP[5]);
    vqf_real_t s3 = fabs(vqf->state.biasP[6])+fabs(vqf->state.biasP[7])+fabs(vqf->state.biasP[8]);
    vqf_real_t P = VQF_MIN(VQF_MAX(VQF_MAX(s1,s2),s3), vqf->coeffs.biasP0);
#else
    vqf_real_t P = vqf->state.biasP;
#endif
    return sqrt(P)*(vqf_real_t)(VQF_PI/100.0/180.0);
}

void vqf_set_bias_estimate(VQF *vqf, vqf_real_t bias[3], vqf_real_t sigma)
{
    memcpy(vqf->state.bias, bias, 3*sizeof(vqf_real_t));
    if (sigma > 0) {
        vqf_real_t P = square(sigma*(vqf_real_t)(180.0*100.0/VQF_PI));
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
        vqf_matrix3_set_to_scaled_identity(P, vqf->state.biasP);
#else
        vqf->state.biasP = P;
#endif
    }
}

bool vqf_get_rest_detected(const VQF *vqf)     { return vqf->state.restDetected; }
bool vqf_get_mag_dist_detected(const VQF *vqf)  { return vqf->state.magDistDetected; }

void vqf_get_relative_rest_deviations(const VQF *vqf, vqf_real_t out[2])
{
    out[0] = sqrt(vqf->state.restLastSquaredDeviations[0]) / (vqf->params.restThGyr*(vqf_real_t)(VQF_PI/180.0));
    out[1] = sqrt(vqf->state.restLastSquaredDeviations[1]) / vqf->params.restThAcc;
}

vqf_real_t vqf_get_mag_ref_norm(const VQF *vqf) { return vqf->state.magRefNorm; }
vqf_real_t vqf_get_mag_ref_dip(const VQF *vqf)  { return vqf->state.magRefDip; }

void vqf_set_mag_ref(VQF *vqf, vqf_real_t norm, vqf_real_t dip)
{
    vqf->state.magRefNorm = norm;
    vqf->state.magRefDip  = dip;
}

void vqf_set_tau_acc(VQF *vqf, vqf_real_t tauAcc)
{
    if (vqf->params.tauAcc == tauAcc) return;
    vqf->params.tauAcc = tauAcc;
    double nB[3], nA[2];
    vqf_filter_coeffs(tauAcc, vqf->coeffs.accTs, nB, nA);
    vqf_filter_adapt_state_for_coeff_change(vqf->state.lastAccLp, 3,
        vqf->coeffs.accLpB, vqf->coeffs.accLpA, nB, nA, vqf->state.accLpState);
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    {
        vqf_real_t R[9]; for (size_t i=0;i<9;i++) R[i]=(vqf_real_t)vqf->state.motionBiasEstRLpState[2*i];
        vqf_filter_adapt_state_for_coeff_change(R, 9, vqf->coeffs.accLpB, vqf->coeffs.accLpA,
            nB, nA, vqf->state.motionBiasEstRLpState);
        vqf_real_t bL[2]; for (size_t i=0;i<2;i++) bL[i]=(vqf_real_t)vqf->state.motionBiasEstBiasLpState[2*i];
        vqf_filter_adapt_state_for_coeff_change(bL, 2, vqf->coeffs.accLpB, vqf->coeffs.accLpA,
            nB, nA, vqf->state.motionBiasEstBiasLpState);
    }
#endif
    memcpy(vqf->coeffs.accLpB, nB, sizeof nB);
    memcpy(vqf->coeffs.accLpA, nA, sizeof nA);
}

void vqf_set_tau_mag(VQF *vqf, vqf_real_t tauMag)
{
    vqf->params.tauMag = tauMag;
    vqf->coeffs.kMag   = vqf_gain_from_tau(tauMag, vqf->coeffs.magTs);
}

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
void vqf_set_motion_bias_est_enabled(VQF *vqf, bool enabled)
{
    if (vqf->params.motionBiasEstEnabled == enabled) return;
    vqf->params.motionBiasEstEnabled = enabled;
    fill_double(vqf->state.motionBiasEstRLpState,    18, VQF_NAN_VAL);
    fill_double(vqf->state.motionBiasEstBiasLpState,  4, VQF_NAN_VAL);
}
#endif

void vqf_set_rest_bias_est_enabled(VQF *vqf, bool enabled)
{
    if (vqf->params.restBiasEstEnabled == enabled) return;
    vqf->params.restBiasEstEnabled = enabled;
    vqf->state.restDetected = false;
    fill_real(vqf->state.restLastSquaredDeviations, 2, 0);
    vqf->state.restT = 0;
    fill_real(vqf->state.restLastGyrLp, 3, 0);
    fill_double(vqf->state.restGyrLpState, 6, VQF_NAN_VAL);
    fill_real(vqf->state.restLastAccLp, 3, 0);
    fill_double(vqf->state.restAccLpState, 6, VQF_NAN_VAL);
}

void vqf_set_mag_dist_rejection_enabled(VQF *vqf, bool enabled)
{
    if (vqf->params.magDistRejectionEnabled == enabled) return;
    vqf->params.magDistRejectionEnabled = enabled;
    vqf->state.magDistDetected  = true;
    vqf->state.magRefNorm       = 0;
    vqf->state.magRefDip        = 0;
    vqf->state.magUndisturbedT  = 0;
    vqf->state.magRejectT       = vqf->params.magMaxRejectionTime;
    vqf->state.magCandidateNorm = -1;
    vqf->state.magCandidateDip  = 0;
    vqf->state.magCandidateT    = 0;
    fill_double(vqf->state.magNormDipLpState, 4, VQF_NAN_VAL);
}

void vqf_set_rest_detection_thresholds(VQF *vqf, vqf_real_t thGyr, vqf_real_t thAcc)
{
    vqf->params.restThGyr = thGyr;
    vqf->params.restThAcc = thAcc;
}

const VQFParams *vqf_get_params(const VQF *vqf)          { return &vqf->params; }
const VQFCoefficients *vqf_get_coeffs(const VQF *vqf)    { return &vqf->coeffs; }
const VQFState *vqf_get_state(const VQF *vqf)            { return &vqf->state; }
void vqf_set_state(VQF *vqf, const VQFState *st)         { vqf->state = *st; }

void vqf_reset_state(VQF *vqf)
{
    VQFState *s = &vqf->state;
    vqf_quat_set_to_identity(s->gyrQuat);
    vqf_quat_set_to_identity(s->accQuat);
    s->delta = 0;
    s->restDetected = false;
    s->magDistDetected = true;
    fill_real(s->lastAccLp, 3, 0);
    fill_double(s->accLpState, 6, VQF_NAN_VAL);
    s->lastAccCorrAngularRate = 0;
    s->kMagInit = 1.0;
    s->lastMagDisAngle = 0;
    s->lastMagCorrAngularRate = 0;
    fill_real(s->bias, 3, 0);
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    vqf_matrix3_set_to_scaled_identity(vqf->coeffs.biasP0, s->biasP);
    fill_double(s->motionBiasEstRLpState,   18, VQF_NAN_VAL);
    fill_double(s->motionBiasEstBiasLpState, 4, VQF_NAN_VAL);
#else
    s->biasP = vqf->coeffs.biasP0;
#endif
    fill_real(s->restLastSquaredDeviations, 2, 0);
    s->restT = 0;
    fill_real(s->restLastGyrLp, 3, 0);
    fill_double(s->restGyrLpState, 6, VQF_NAN_VAL);
    fill_real(s->restLastAccLp, 3, 0);
    fill_double(s->restAccLpState, 6, VQF_NAN_VAL);
    s->magRefNorm = 0;
    s->magRefDip  = 0;
    s->magUndisturbedT = 0;
    s->magRejectT = vqf->params.magMaxRejectionTime;
    s->magCandidateNorm = -1;
    s->magCandidateDip  = 0;
    s->magCandidateT    = 0;
    fill_real(s->magNormDip, 2, 0);
    fill_double(s->magNormDipLpState, 4, VQF_NAN_VAL);
}
