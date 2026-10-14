/**
 * @file adcs_sgp4.h
 * @brief Near-Earth SGP4 orbit propagator (Vallado revision, WGS-72).
 *
 * Line-by-line port of the near-Earth branch of sgp4init()/sgp4() from
 * D. Vallado's C++ reference implementation (AIAA 2006-6753, 2020/2021
 * revision distributed by CelesTrak). Deep-space (SDP4, period ≥ 225 min)
 * element sets are rejected at initialisation; a LEO PocketQube never needs
 * them.
 *
 * Differences from the reference, none of which change the numbers:
 *   - gravity constants are fixed to WGS-72 (TLEs are generated with them);
 *   - no globals, no printf; the record is caller-allocated and
 *     adcs_sgp4_propagate() does not modify it (re-entrant);
 *   - epoch bookkeeping (Julian dates) is left to the caller — the
 *     propagator only needs minutes since epoch.
 *
 * Output frame: TEME (True Equator, Mean Equinox of date).
 */

#ifndef ADCS_SGP4_H
#define ADCS_SGP4_H

#ifdef __cplusplus
extern "C" {
#endif

#include "adcs_tle.h"

/** SGP4 return codes (1–6 keep Vallado's satrec.error numbering) */
typedef enum {
    ADCS_SGP4_OK              = 0,
    ADCS_SGP4_ERR_ECC         = 1,  /**< Mean eccentricity outside [-0.001, 1) */
    ADCS_SGP4_ERR_MEAN_MOTION = 2,  /**< Mean motion ≤ 0 */
    ADCS_SGP4_ERR_PERT_ECC    = 3,  /**< Perturbed eccentricity out of range (deep space only) */
    ADCS_SGP4_ERR_SEMILATUS   = 4,  /**< Semi-latus rectum < 0 */
    ADCS_SGP4_ERR_SUBORBITAL  = 5,  /**< Epoch elements sub-orbital (unused, kept for numbering) */
    ADCS_SGP4_ERR_DECAYED     = 6,  /**< Radius < 1 earth radius: satellite has decayed */
    ADCS_SGP4_ERR_DEEP_SPACE  = 10, /**< Period ≥ 225 min: SDP4 required, not supported */
    ADCS_SGP4_ERR_INVALID     = 11  /**< NULL pointer or invalid elements */
} adcs_sgp4_status_t;

/** Initialised near-Earth SGP4 record (subset of Vallado's elsetrec) */
typedef struct {
    /* Mean elements at epoch (SGP4 units: rad, rad/min) */
    double bstar, ecco, argpo, inclo, mo, nodeo;
    double no_unkozai;

    /* Near-Earth secular / drag / periodic coefficients */
    int    isimp;
    double aycof, con41, cc1, cc4, cc5;
    double d2, d3, d4, delmo, eta;
    double argpdot, omgcof, sinmao;
    double t2cof, t3cof, t4cof, t5cof;
    double x1mth2, x7thm1, mdot, nodedot;
    double xlcof, xmcof, nodecf;
} adcs_sgp4_t;

/**
 * @brief Initialise an SGP4 record from parsed TLE elements.
 *
 * Does not propagate; call adcs_sgp4_propagate(rec, 0, …) to check that
 * the element set produces a valid state at epoch.
 *
 * @param[in]  tle  Parsed TLE (see adcs_tle_parse())
 * @param[out] rec  Initialised record
 * @return ADCS_SGP4_OK, ADCS_SGP4_ERR_DEEP_SPACE or ADCS_SGP4_ERR_INVALID
 */
int adcs_sgp4_init(const adcs_tle_t *tle, adcs_sgp4_t *rec);

/**
 * @brief Propagate to a time offset from the TLE epoch.
 *
 * @param[in]  rec         Record from adcs_sgp4_init()
 * @param[in]  tsince_min  Minutes since the TLE epoch (may be negative)
 * @param[out] r_km        Position in TEME [km]
 * @param[out] v_kms       Velocity in TEME [km/s]
 * @return ADCS_SGP4_OK or an adcs_sgp4_status_t error code. On errors 1, 2
 *         and 4 the outputs are not written; on error 6 they hold the
 *         (sub-surface) state.
 */
int adcs_sgp4_propagate(const adcs_sgp4_t *rec, double tsince_min,
                        double r_km[3], double v_kms[3]);

#ifdef __cplusplus
}
#endif

#endif /* ADCS_SGP4_H */
