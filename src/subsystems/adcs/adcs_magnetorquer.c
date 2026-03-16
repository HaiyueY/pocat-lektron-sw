/**
 * @file adcs_magnetorquer.c
 * @brief Magnetorquer dipole-to-intensity conversion and quantization.
 *
 * Implements the BD2606MVV driver command pipeline:
 *   dipole → clamp → intensity → quantize → recompute dipole
 *
 * MATLAB reference:
 *   Detumbling.m  L144-184  (intensity quantization loop)
 *   Nadir_pointing.m L196-238 (same quantization scheme)
 */

#include "adcs_magnetorquer.h"
#include "adcs_config.h"
#include <math.h>

/* Coil factors: N * S per axis [turns·m²] */
static const double coil_factor[3] = {
    MTQ_COIL_FACTOR_X,
    MTQ_COIL_FACTOR_Y,
    MTQ_COIL_FACTOR_Z
};

/* Maximum dipole per axis [A·m²] */
static const double max_dipole[3] = {
    MTQ_MAX_DIPOLE_X,
    MTQ_MAX_DIPOLE_Y,
    MTQ_MAX_DIPOLE_Z
};

/**
 * Quantize a value to the nearest step within [min, max].
 */
static double quantize_intensity(double raw_ma)
{
    double sign_val = (raw_ma >= 0.0) ? 1.0 : -1.0;
    double abs_val = fabs(raw_ma);

    /* Quantize to nearest step */
    double quantized = round(abs_val / MTQ_INTENSITY_STEP_MA) * MTQ_INTENSITY_STEP_MA;

    /* Clamp to [min, max] range */
    if (quantized < MTQ_MIN_INTENSITY_MA) {
        quantized = 0.0;  /* Below minimum: turn off */
    }
    if (quantized > MTQ_MAX_INTENSITY_MA) {
        quantized = MTQ_MAX_INTENSITY_MA;
    }

    return sign_val * quantized;
}

void mtq_compute_command(vec3d_t desired_dipole, mtq_command_t *cmd)
{
    double dipole_arr[3] = {desired_dipole.x, desired_dipole.y, desired_dipole.z};
    double intensity_arr[3];
    int i;

    /* Step 1: Clamp dipole to hardware limits */
    for (i = 0; i < 3; i++) {
        if (fabs(dipole_arr[i]) > max_dipole[i]) {
            dipole_arr[i] = (dipole_arr[i] >= 0.0) ? max_dipole[i] : -max_dipole[i];
        }
    }

    /* Step 2: Convert dipole to continuous intensity [mA] */
    for (i = 0; i < 3; i++) {
        intensity_arr[i] = (dipole_arr[i] / coil_factor[i]) * 1.0e3;
    }

    /* Step 3: Quantize intensity */
    for (i = 0; i < 3; i++) {
        intensity_arr[i] = quantize_intensity(intensity_arr[i]);
    }

    /* Step 4: Recompute dipole from quantized intensity for consistency */
    for (i = 0; i < 3; i++) {
        dipole_arr[i] = (intensity_arr[i] / 1.0e3) * coil_factor[i];
    }

    /* Pack output */
    cmd->dipole.x = dipole_arr[0];
    cmd->dipole.y = dipole_arr[1];
    cmd->dipole.z = dipole_arr[2];
    cmd->intensity_ma.x = intensity_arr[0];
    cmd->intensity_ma.y = intensity_arr[1];
    cmd->intensity_ma.z = intensity_arr[2];
}

/* Default implementation: no-op (overridden by HAL or simulator) */
__attribute__((weak))
int mtq_apply_command(const mtq_command_t *cmd)
{
    (void)cmd;
    return 0;
}
