/**
 * @file adcs_math.h
 * @brief Vector, quaternion, and matrix math utilities for ADCS.
 *
 * Provides lightweight 3D math operations using the adcs_types.h
 * vec3d_t, quat_t, and mat3d_t types. These wrap and extend the
 * existing help_adcs.h functions with a type-safe interface.
 */

#ifndef ADCS_MATH_H
#define ADCS_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "adcs_types.h"
#include <math.h>

/* ---- Vector operations ------------------------------------------------ */

/** Return zero vector */
static inline vec3d_t vec3d_zero(void)
{
    vec3d_t v = {0.0, 0.0, 0.0};
    return v;
}

/** Create vector from components */
static inline vec3d_t vec3d_make(double x, double y, double z)
{
    vec3d_t v = {x, y, z};
    return v;
}

/** Vector addition: a + b */
static inline vec3d_t vec3d_add(vec3d_t a, vec3d_t b)
{
    vec3d_t r = {a.x + b.x, a.y + b.y, a.z + b.z};
    return r;
}

/** Vector subtraction: a - b */
static inline vec3d_t vec3d_sub(vec3d_t a, vec3d_t b)
{
    vec3d_t r = {a.x - b.x, a.y - b.y, a.z - b.z};
    return r;
}

/** Scalar multiplication: s * v */
static inline vec3d_t vec3d_scale(vec3d_t v, double s)
{
    vec3d_t r = {v.x * s, v.y * s, v.z * s};
    return r;
}

/** Dot product: a · b */
static inline double vec3d_dot(vec3d_t a, vec3d_t b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/** Cross product: a × b */
static inline vec3d_t vec3d_cross(vec3d_t a, vec3d_t b)
{
    vec3d_t r;
    r.x = a.y * b.z - a.z * b.y;
    r.y = a.z * b.x - a.x * b.z;
    r.z = a.x * b.y - a.y * b.x;
    return r;
}

/** Euclidean norm: ||v|| */
static inline double vec3d_norm(vec3d_t v)
{
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

/** Normalize vector to unit length */
static inline vec3d_t vec3d_normalize(vec3d_t v)
{
    double n = vec3d_norm(v);
    if (n < 1.0e-15) {
        return vec3d_zero();
    }
    return vec3d_scale(v, 1.0 / n);
}

/* ---- Quaternion operations -------------------------------------------- */

/** Identity quaternion [1, 0, 0, 0] */
static inline quat_t quat_identity(void)
{
    quat_t q = {1.0, 0.0, 0.0, 0.0};
    return q;
}

/** Quaternion norm */
static inline double quat_norm(quat_t q)
{
    return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

/** Normalize quaternion to unit length */
static inline quat_t quat_normalize(quat_t q)
{
    double n = quat_norm(q);
    if (n < 1.0e-15) {
        return quat_identity();
    }
    quat_t r = {q.w / n, q.x / n, q.y / n, q.z / n};
    return r;
}

/** Quaternion conjugate (inverse for unit quaternion) */
static inline quat_t quat_conjugate(quat_t q)
{
    quat_t r = {q.w, -q.x, -q.y, -q.z};
    return r;
}

/**
 * Quaternion multiplication: q1 * q2
 * If q1 rotates from B→C and q2 from A→B, result rotates A→C.
 */
static inline quat_t quat_multiply(quat_t q1, quat_t q2)
{
    quat_t r;
    r.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    r.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    r.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    r.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return r;
}

/** Enforce positive scalar part convention */
static inline quat_t quat_positive_scalar(quat_t q)
{
    if (q.w < 0.0) {
        q.w = -q.w;
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
    }
    return q;
}

/** Extract vector part of quaternion */
static inline vec3d_t quat_vector_part(quat_t q)
{
    vec3d_t v = {q.x, q.y, q.z};
    return v;
}

/**
 * Rotate vector by quaternion: v' = q * v * q*
 */
static inline vec3d_t quat_rotate_vec(quat_t q, vec3d_t v)
{
    /* Using the formula: v' = v + 2*w*(e×v) + 2*(e×(e×v))
     * where q = [w, e] */
    vec3d_t e = {q.x, q.y, q.z};
    vec3d_t ev = vec3d_cross(e, v);
    vec3d_t eev = vec3d_cross(e, ev);
    vec3d_t r;
    r.x = v.x + 2.0 * (q.w * ev.x + eev.x);
    r.y = v.y + 2.0 * (q.w * ev.y + eev.y);
    r.z = v.z + 2.0 * (q.w * ev.z + eev.z);
    return r;
}

/**
 * Compute quaternion from two unit vectors: rotation from v1 to v2
 * Equivalent to MATLAB U2Q function.
 */
static inline quat_t quat_from_two_vectors(vec3d_t v1, vec3d_t v2)
{
    double d = vec3d_dot(v1, v2);
    quat_t q;

    if (d < -0.999999) {
        /* Vectors are anti-parallel: use arbitrary perpendicular axis */
        vec3d_t aux = {0.0, 0.0, 1.0};
        vec3d_t cp = vec3d_cross(aux, v2);
        double cp_norm = vec3d_norm(cp);
        if (cp_norm < 1.0e-10) {
            aux.x = 1.0; aux.y = 0.0; aux.z = 0.0;
            cp = vec3d_cross(aux, v2);
        }
        cp = vec3d_normalize(cp);
        q.w = 0.0;
        q.x = cp.x;
        q.y = cp.y;
        q.z = cp.z;
    } else {
        vec3d_t cp = vec3d_cross(v1, v2);
        double s = sqrt(2.0 * (1.0 + d));
        q.w = 0.5 * s;
        q.x = cp.x / s;
        q.y = cp.y / s;
        q.z = cp.z / s;
    }

    return quat_normalize(q);
}

/**
 * Convert rotation matrix to quaternion.
 * Uses Shepperd's method for numerical stability.
 */
static inline quat_t quat_from_matrix(mat3d_t m)
{
    quat_t q;
    double trace = m.m[0][0] + m.m[1][1] + m.m[2][2];

    if (trace > 0.0) {
        double s = 0.5 / sqrt(trace + 1.0);
        q.w = 0.25 / s;
        q.x = (m.m[2][1] - m.m[1][2]) * s;
        q.y = (m.m[0][2] - m.m[2][0]) * s;
        q.z = (m.m[1][0] - m.m[0][1]) * s;
    } else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2]) {
        double s = 2.0 * sqrt(1.0 + m.m[0][0] - m.m[1][1] - m.m[2][2]);
        q.w = (m.m[2][1] - m.m[1][2]) / s;
        q.x = 0.25 * s;
        q.y = (m.m[0][1] + m.m[1][0]) / s;
        q.z = (m.m[0][2] + m.m[2][0]) / s;
    } else if (m.m[1][1] > m.m[2][2]) {
        double s = 2.0 * sqrt(1.0 + m.m[1][1] - m.m[0][0] - m.m[2][2]);
        q.w = (m.m[0][2] - m.m[2][0]) / s;
        q.x = (m.m[0][1] + m.m[1][0]) / s;
        q.y = 0.25 * s;
        q.z = (m.m[1][2] + m.m[2][1]) / s;
    } else {
        double s = 2.0 * sqrt(1.0 + m.m[2][2] - m.m[0][0] - m.m[1][1]);
        q.w = (m.m[1][0] - m.m[0][1]) / s;
        q.x = (m.m[0][2] + m.m[2][0]) / s;
        q.y = (m.m[1][2] + m.m[2][1]) / s;
        q.z = 0.25 * s;
    }

    return quat_normalize(q);
}

/**
 * Propagate quaternion forward using gyro angular velocity.
 *
 * The simulation uses kinematic equation (left multiplication):
 *   dq/dt = -0.5 * [0, ω_body] ⊗ q
 * Closed-form integration over dt:
 *   q(t+dt) = dq ⊗ q(t)            (LEFT multiply)
 *   dq = [cos(θ/2), -ω̂·sin(θ/2)],  θ = |ω|·dt
 */
static inline quat_t quat_gyro_propagate(quat_t q, vec3d_t omega, double dt)
{
    double w_norm = vec3d_norm(omega);
    double theta = w_norm * dt;
    quat_t dq;
    if (theta < 1.0e-12) {
        dq = quat_identity();
    } else {
        double half_theta = 0.5 * theta;
        double c = cos(half_theta);
        double k = -sin(half_theta) / w_norm;
        dq.w = c;
        dq.x = k * omega.x;
        dq.y = k * omega.y;
        dq.z = k * omega.z;
    }
    /* LEFT multiply: q' = dq ⊗ q */
    return quat_normalize(quat_multiply(dq, q));
}

#ifdef __cplusplus
}
#endif

#endif /* ADCS_MATH_H */
