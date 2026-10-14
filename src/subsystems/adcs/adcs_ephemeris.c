/**
 * @file adcs_ephemeris.c
 * @brief On-board reference vectors: orbit, geomagnetic field and Sun.
 *
 * Per evaluation: one SGP4 propagation, one GMST, one IGRF synthesis
 * (degree igrf_order) and one solar ephemeris. See adcs_ephemeris.h for
 * the frame conventions.
 */

#include "adcs_ephemeris.h"
#include "adcs_config.h"
#include "adcs_igrf.h"
#include "adcs_math.h"
#include "adcs_sun.h"
#include "adcs_time.h"
#include <math.h>

void adcs_orbit_init(adcs_orbit_t *orbit)
{
    if (orbit == 0) {
        return;
    }
    adcs_orbit_t empty = {0};
    *orbit = empty;
}

int adcs_orbit_load_tle(adcs_orbit_t *orbit, const char *line1, const char *line2)
{
    adcs_tle_t tle;
    adcs_sgp4_t rec;
    double r_km[3], v_kms[3];

    if (orbit == 0 || line1 == 0 || line2 == 0) {
        return ADCS_ORBIT_ERR_NULL;
    }
    if (adcs_tle_parse(line1, line2, &tle) != ADCS_TLE_OK) {
        return ADCS_ORBIT_ERR_PARSE;
    }
    if (adcs_sgp4_init(&tle, &rec) != ADCS_SGP4_OK) {
        return ADCS_ORBIT_ERR_INIT;
    }
    if (adcs_sgp4_propagate(&rec, 0.0, r_km, v_kms) != ADCS_SGP4_OK) {
        return ADCS_ORBIT_ERR_EPOCH;
    }

    orbit->tle = tle;
    orbit->sgp4 = rec;
    orbit->loaded = 1;
    return ADCS_ORBIT_OK;
}

uint8_t adcs_ephem_update(const adcs_orbit_t *orbit, double unix_s,
                          int igrf_order, adcs_ephem_t *eph)
{
    double r_km[3], v_kms[3];
    uint8_t status = ADCS_EPH_OK;

    if (eph == 0) {
        return ADCS_EPH_NO_TLE;
    }

    eph->unix_s = unix_s;
    eph->tle_age_s = 0.0;
    eph->pos_eci = vec3d_zero();
    eph->vel_eci = vec3d_zero();
    eph->nadir_eci = vec3d_zero();
    eph->b_field_eci = vec3d_zero();
    eph->sun_eci = vec3d_zero();
    eph->eclipse = 0;
    eph->sgp4_error = ADCS_SGP4_OK;

    if (orbit == 0 || !orbit->loaded) {
        eph->status = ADCS_EPH_NO_TLE;
        return eph->status;
    }

    /* ---- Orbit (TEME) ---- */
    eph->tle_age_s = unix_s - orbit->tle.epoch_unix;
    if (fabs(eph->tle_age_s) > ADCS_TLE_MAX_AGE_S) {
        status |= ADCS_EPH_TLE_STALE;
    }

    int rc = adcs_sgp4_propagate(&orbit->sgp4, eph->tle_age_s / 60.0, r_km, v_kms);
    eph->sgp4_error = (int8_t)rc;
    if (rc != ADCS_SGP4_OK) {
        eph->status = status | ADCS_EPH_SGP4_ERROR;
        return eph->status;
    }

    eph->pos_eci = vec3d_make(r_km[0] * 1.0e3, r_km[1] * 1.0e3, r_km[2] * 1.0e3);
    eph->vel_eci = vec3d_make(v_kms[0] * 1.0e3, v_kms[1] * 1.0e3, v_kms[2] * 1.0e3);
    eph->nadir_eci = vec3d_scale(vec3d_normalize(eph->pos_eci), -1.0);

    /* ---- Geomagnetic field: TEME → PEF → IGRF → TEME ---- */
    double gmst = adcs_time_gmst(unix_s);
    vec3d_t r_ecef = vec3d_rot_z(eph->pos_eci, gmst);
    vec3d_t b_ecef;
    rc = adcs_igrf_field_ecef(r_ecef, adcs_time_unix_to_decimal_year(unix_s),
                              igrf_order, &b_ecef);
    if (rc == ADCS_IGRF_ERR_INVALID) {
        status |= ADCS_EPH_IGRF_ERROR;
    } else {
        if (rc == ADCS_IGRF_EXTRAPOLATED) {
            status |= ADCS_EPH_IGRF_EXTRAPOLATED;
        }
        eph->b_field_eci = vec3d_rot_z(b_ecef, -gmst);
    }

    /* ---- Sun and shadow ---- */
    eph->sun_eci = adcs_sun_direction(unix_s);
    eph->eclipse = adcs_eclipse_cylindrical(eph->pos_eci, eph->sun_eci);

    eph->status = status;
    return status;
}

int adcs_ephem_to_state(const adcs_ephem_t *eph, adcs_state_t *state)
{
    if (eph == 0 || state == 0 || (eph->status & ADCS_EPH_INVALID_MASK) != 0) {
        return -1;
    }
    state->pos_eci = eph->pos_eci;
    state->vel_eci = eph->vel_eci;
    state->nadir_eci = eph->nadir_eci;
    state->b_field_eci = eph->b_field_eci;
    state->sun_eci = eph->sun_eci;
    return 0;
}
