/**
 * @file adcs_determination.c
 * @brief TRIAD attitude determination algorithm implementation.
 *
 * The TRIAD algorithm estimates the rotation matrix from ECI (reference)
 * frame to body frame using two non-parallel vector pairs measured in
 * both frames.
 *
 * Uses the (Sun, B) vector pairs, so it is only meaningful while the Sun
 * is visible; eclipse attitude is propagated by nadir_step().
 *
 * MATLAB reference:
 *   ref/H-bridge-simulations/Simulation Nadir Pointing/Functions/triad.m
 *     - Builds orthonormal triads from two vector pairs
 *     - rotation_matrix = M_ref * M_body'
 *
 *   ref/H-bridge-simulations/Simulation Nadir Pointing/Nadir_pointing.m
 *     L112-119: Sun-visible TRIAD (reference1=B_ECI, body1=B_body,
 *               reference2=Sun_ECI, body2=Sun_body)
 *               Called as triad(Sun_ECI, B_ECI, Sun_body, B_body)
 *     L126-140: Eclipse TRIAD on (B, dB/dt) — not implemented here
 *     L123-144: rotmatrix2quat → Shepperd's method (quat_from_matrix)
 */

#include "adcs_determination.h"
#include "adcs_config.h"
#include "adcs_math.h"
#include <math.h>

/**
 * @brief Build an orthonormal triad from two non-parallel vectors.
 *
 * MATLAB triad.m builds:
 *   t1 = v1 / ||v1||
 *   t2 = cross(v1, v2) / ||cross(v1, v2)||
 *   t3 = cross(v1, t2) / ||cross(v1, t2)||
 *
 * @param[in]  v1  Primary vector (first arg to MATLAB triad)
 * @param[in]  v2  Secondary vector (second arg to MATLAB triad)
 * @param[out] t1  First triad axis (output)
 * @param[out] t2  Second triad axis (output)
 * @param[out] t3  Third triad axis (output)
 */
static void build_triad(vec3d_t v1, vec3d_t v2,
                         vec3d_t *t1, vec3d_t *t2, vec3d_t *t3)
{
    *t1 = vec3d_normalize(v1);
    *t2 = vec3d_normalize(vec3d_cross(v1, v2));
    *t3 = vec3d_normalize(vec3d_cross(v1, *t2));
}

void triad_compute(vec3d_t ref1, vec3d_t ref2,
                   vec3d_t body1, vec3d_t body2,
                   mat3d_t *rot)
{
    /* Build orthonormal triads in reference and body frames
     * MATLAB: aux1 = [v1 v2 v3], aux2 = [w1 w2 w3]'
     *         rotation_matrix = aux1 * aux2 */
    vec3d_t v1, v2, v3;
    vec3d_t w1, w2, w3;

    build_triad(ref1, ref2, &v1, &v2, &v3);
    build_triad(body1, body2, &w1, &w2, &w3);

    /* rotation_matrix = M_ref * M_body^T
     * M_ref has columns [v1 v2 v3], M_body has columns [w1 w2 w3]
     * So M_ref * M_body^T = sum_k (v_k * w_k^T) */

    /* Column vectors as arrays for indexing */
    double v_cols[3][3] = {
        {v1.x, v2.x, v3.x},
        {v1.y, v2.y, v3.y},
        {v1.z, v2.z, v3.z}
    };
    double w_cols[3][3] = {
        {w1.x, w2.x, w3.x},
        {w1.y, w2.y, w3.y},
        {w1.z, w2.z, w3.z}
    };

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rot->m[i][j] = 0.0;
            for (int k = 0; k < 3; k++) {
                rot->m[i][j] += v_cols[i][k] * w_cols[j][k];
            }
        }
    }
}

/** Body-frame normals of the six CSS faces, in sun_sensor_data_t order */
static const vec3d_t css_normals[6] = {
    { 1.0,  0.0,  0.0},
    {-1.0,  0.0,  0.0},
    { 0.0,  1.0,  0.0},
    { 0.0, -1.0,  0.0},
    { 0.0,  0.0,  1.0},
    { 0.0,  0.0, -1.0}
};

void determination_css_sun(sun_sensor_data_t *sun)
{
    vec3d_t sum = vec3d_zero();
    double v_max = 0.0;

    sun->sun_body = vec3d_zero();
    sun->sun_visible = 0;
    if (!sun->valid) {
        return;
    }

    for (int i = 0; i < 6; i++) {
        double v = sun->voltage[i];
        sum = vec3d_add(sum, vec3d_scale(css_normals[i], v));
        if (v > v_max) {
            v_max = v;
        }
    }

    /* Opposite faces cancel, so a non-zero sum needs at least one lit face */
    if (v_max < ADCS_CSS_SUN_THRESHOLD_V || vec3d_norm(sum) < 1.0e-6) {
        return;
    }
    sun->sun_body = vec3d_normalize(sum);
    sun->sun_visible = 1;
}

void determination_update(adcs_state_t *state)
{
    /* Sun-visible TRIAD
     * Ref: Nadir_pointing.m L112-119
     *   triad(Sun_ECI, B_ECI, Sun_body, B_body)
     *
     * The MATLAB eclipse variant (dB/dt pair, L126-140) is not used: in
     * eclipse nadir_step() propagates the last TRIAD with the
     * bias-corrected gyro instead, which needs no dB_ECI/dt history. */
    vec3d_t ref_primary   = state->sun_eci;
    vec3d_t ref_secondary = state->b_field_eci;
    vec3d_t body_primary   = state->sun.sun_body;
    vec3d_t body_secondary = state->mag.field;

    /* Compute rotation matrix via TRIAD */
    mat3d_t rot;
    triad_compute(ref_primary, ref_secondary, body_primary, body_secondary, &rot);

    /* Convert to quaternion (Shepperd's method).
     *
     * TRIAD builds R = M_ref × M_body^T = R_body_to_ECI
     * (verified: R × v_body = v_ECI for the primary/secondary vector pairs).
     * quat_from_matrix(R_body_to_ECI) therefore returns q_body_to_ECI.
     *
     * The nadir controller's error quaternion expects q_ECI_to_body
     * (see adcs_nadir.c L67-76 and Nadir_pointing.m L156:
     *  q_err = q_target ⊗ conj(q_eci_body), which equals
     *  q_desired ⊗ q_actual^{-1} only when q_eci_body = q_ECI_to_body).
     *
     * Conjugating converts q_body_to_ECI → q_ECI_to_body.
     *
     * See docs/adcs_nadir_triad_convention_analysis.md for full derivation. */
    state->q_eci_body = quat_conjugate(quat_from_matrix(rot));
}
