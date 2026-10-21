/**
 * @file adcs_calib.c
 * @brief Default calibration and the sensor correction models.
 *
 * See adcs_calib.h for the models. Everything here is platform
 * independent; the board HAL calls it after reading the raw sensors.
 */

#include "adcs_calib.h"
#include "adcs_config.h"
#include <string.h>

/** Copper resistance temperature coefficient at 20 °C [1/K] */
#define CU_ALPHA_20C    3.93e-3f

static void identity3(float m[3][3])
{
    int i, j;

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            m[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

static vec3d_t mul3(const float m[3][3], const double v[3])
{
    vec3d_t r;

    r.x = m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2];
    r.y = m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2];
    r.z = m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2];
    return r;
}

void adcs_calib_defaults(adcs_calib_t *cal)
{
    int i;

    memset(cal, 0, sizeof(*cal));
    cal->version = ADCS_CALIB_VERSION;
    cal->size = (uint16_t)sizeof(*cal);

    identity3(cal->mag_soft_iron);
    identity3(cal->mag_to_body);        /* TBC: S2b polarity table */
    identity3(cal->gyro_to_body);       /* TBC: S2b polarity table */
    cal->gyro_bias_t0_c = 25.0f;

    for (i = 0; i < ADCS_CSS_COUNT; i++) {
        cal->css_gain[i] = 1.0f;
    }

    cal->mtq_coil_r_ohm[0] = (float)MTQ_COIL_R_OHM_X;
    cal->mtq_coil_r_ohm[1] = (float)MTQ_COIL_R_OHM_Y;
    cal->mtq_coil_r_ohm[2] = (float)MTQ_COIL_R_OHM_Z;
    cal->mtq_coil_tc = CU_ALPHA_20C;
}

int adcs_calib_check(const adcs_calib_t *cal)
{
    if (cal->version != ADCS_CALIB_VERSION || cal->size != sizeof(*cal)) {
        return -1;
    }
    return 0;
}

vec3d_t adcs_calib_mag(const adcs_calib_t *cal, const double raw_t[3])
{
    double centred[3];
    double corrected[3];
    vec3d_t s;
    int i;

    for (i = 0; i < 3; i++) {
        centred[i] = raw_t[i] - cal->mag_hard_iron[i];
    }
    s = mul3(cal->mag_soft_iron, centred);
    corrected[0] = s.x;
    corrected[1] = s.y;
    corrected[2] = s.z;
    return mul3(cal->mag_to_body, corrected);
}

vec3d_t adcs_calib_gyro(const adcs_calib_t *cal, const double raw_rad_s[3],
                        double temp_c)
{
    double dt = temp_c - cal->gyro_bias_t0_c;
    double corrected[3];
    int i;

    for (i = 0; i < 3; i++) {
        corrected[i] = raw_rad_s[i] - (cal->gyro_bias[i] + cal->gyro_bias_tc[i] * dt);
    }
    return mul3(cal->gyro_to_body, corrected);
}

void adcs_calib_css(const adcs_calib_t *cal, const double raw_v[ADCS_CSS_COUNT],
                    double out_v[ADCS_CSS_COUNT])
{
    int i;

    for (i = 0; i < ADCS_CSS_COUNT; i++) {
        double v = cal->css_gain[i] * (raw_v[i] - cal->css_offset[i]);
        out_v[i] = (v > 0.0) ? v : 0.0;
    }
}

double adcs_calib_coil_r(const adcs_calib_t *cal, int axis, double temp_c)
{
    return cal->mtq_coil_r_ohm[axis] * (1.0 + cal->mtq_coil_tc * (temp_c - 20.0));
}
