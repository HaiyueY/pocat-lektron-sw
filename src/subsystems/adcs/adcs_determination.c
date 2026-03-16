/**
 * @file adcs_determination.c
 * @brief TRIAD attitude determination algorithm implementation.
 *
 * The TRIAD algorithm estimates the rotation matrix from ECI (reference)
 * frame to body frame using two non-parallel vector pairs measured in
 * both frames.
 *
 * Two modes of operation:
 *   - Sun-visible: uses (B_ECI, Sun_ECI) and (B_body, Sun_body)
 *   - Eclipse:     uses (B_ECI, dB_ECI/dt) and (B_body, dB_body/dt + ω×B_body)
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
 *     L126-140: Eclipse TRIAD (reference1=B_ECI, body1=B_body,
 *               reference2=dB_ECI/dt, body2=dB_body/dt + ω×B_body)
 *               Called as triad(dB_ECI/dt, B_ECI, dB_body_corr, B_body)
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

void determination_update(adcs_state_t *state)
{
    vec3d_t ref_primary, ref_secondary;
    vec3d_t body_primary, body_secondary;

    if (state->sun.sun_visible) {
        /* Sun-visible mode
         * Ref: Nadir_pointing.m L112-119
         *   triad(Sun_ECI, B_ECI, Sun_body, B_body) */
        ref_primary  = state->sun_eci;
        ref_secondary = state->b_field_eci;
        body_primary = state->sun.sun_body;
        body_secondary = state->mag.field;
    } else {
        /* Eclipse mode
         * Ref: Nadir_pointing.m L126-140
         *   reference2 = (B_ECI - B_ECI_prev) / dT
         *   body2 = dB_body/dt + ω × B_body
         *   triad(dB_ECI/dt, B_ECI, body2, B_body) */
        double dt = state->dt;

        /* ECI magnetic field derivative */
        vec3d_t db_eci;
        db_eci.x = (state->b_field_eci.x - state->b_field_eci_prev.x) / dt;
        db_eci.y = (state->b_field_eci.y - state->b_field_eci_prev.y) / dt;
        db_eci.z = (state->b_field_eci.z - state->b_field_eci_prev.z) / dt;

        /* Guard against zero derivative (first step) */
        if (vec3d_norm(db_eci) < 1.0e-15) {
            db_eci = vec3d_make(1.0e-9, 1.0e-9, 1.0e-9);
        }

        /* Body magnetic field derivative */
        vec3d_t db_body;
        db_body.x = (state->mag.field.x - state->mag_field_prev.x) / dt;
        db_body.y = (state->mag.field.y - state->mag_field_prev.y) / dt;
        db_body.z = (state->mag.field.z - state->mag_field_prev.z) / dt;

        if (vec3d_norm(db_body) < 1.0e-15) {
            db_body = vec3d_make(1.0e-9, 1.0e-9, 1.0e-9);
        }

        /* Corrected body derivative: dB_body/dt + ω × B_body
         * Ref: Nadir_pointing.m L139 */
        vec3d_t omega_cross_b = vec3d_cross(state->gyro.angular_vel,
                                            state->mag.field);
        vec3d_t body2_corrected = vec3d_add(db_body, omega_cross_b);

        ref_primary  = db_eci;
        ref_secondary = state->b_field_eci;
        body_primary = body2_corrected;
        body_secondary = state->mag.field;
    }

    /* Compute rotation matrix via TRIAD */
    mat3d_t rot;
    triad_compute(ref_primary, ref_secondary, body_primary, body_secondary, &rot);

    /* Convert to quaternion (Shepperd's method) */
    state->q_eci_body = quat_from_matrix(rot);
}
