/**
 * @file adcs_magnetorquer.c
 * @brief Magnetorquer dipole-to-intensity conversion and quantization.
 *
 * Implements the BD2606MVV driver command pipeline:
 *   dipole → clamp → intensity → first-order Σ-Δ dither → recompute dipole
 *
 * MATLAB reference:
 *   Detumbling.m  L144-184  (intensity quantization loop)
 *   Nadir_pointing.m L196-238 (same quantization scheme)
 *
 * Σ-Δ dithering rationale (this file's only deviation from MATLAB):
 *   The driver hardware can only output current at multiples of
 *   MTQ_INTENSITY_STEP_MA (0.5 mA), with everything below
 *   MTQ_MIN_INTENSITY_MA (also 0.5 mA) snapped to zero — i.e. there
 *   is a true deadband.  The MATLAB reference paper [1] solves this
 *   for nadir-pointing by using a "fixed" magnetorquer model with PWM
 *   to time-average sub-step commands.  We implement the same idea as
 *   a per-axis 1st-order Σ-Δ noise-shaping accumulator: the residual
 *   between the continuous command and the nearest quantized step is
 *   accumulated; once |accumulator| crosses one step, the next sample
 *   bumps the output up by one step.  Average output = continuous
 *   command, but the actuator only ever fires at addressable values.
 *
 *   This is essential for nadir steady-state, where PD output is
 *   typically 0.05-0.3 mA — without dither it would be deadbanded to
 *   zero, leaving sensor noise to control the actuator (severe limit
 *   cycle, last-orbit error ~14°).  With dither, sub-step commands
 *   are accurately reproduced as time averages.
 *
 *   [1] Sugimura et al., "Attitude Determination and Control System
 *       for Nadir Pointing Using Magnetorquer and Magnetometer",
 *       IEEE Aerospace 2016, §5 ("fixed" magnetorquer model).
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

/* Per-axis Σ-Δ accumulators [mA], persistent across calls.
 * Bounded by |accumulator| <= step/2 in steady state. */
static double sigma_delta_accumulator[3] = {0.0, 0.0, 0.0};

void mtq_reset_accumulator(void)
{
    sigma_delta_accumulator[0] = 0.0;
    sigma_delta_accumulator[1] = 0.0;
    sigma_delta_accumulator[2] = 0.0;
}

/**
 * 1st-order Σ-Δ quantizer for one axis.
 *   target = raw + accumulator       (include past error)
 *   q = round(target / step) * step  (nearest addressable level)
 *   accumulator = target - q         (carry residual forward)
 *
 * Deadband (output < MIN) is enforced naturally because MIN == step
 * for this hardware: q is always either 0 or |q| >= step = MIN.
 *
 * Saturation: if |target| would exceed MAX, the accumulator stops
 * winding up (anti-windup) so it doesn't take many cycles to
 * recover after a saturated command.
 */
static double sigma_delta_step(double raw_ma, double *accumulator)
{
    double target = raw_ma + *accumulator;

    /* Anti-windup saturation: clamp target before quantizing */
    if (target > MTQ_MAX_INTENSITY_MA) target = MTQ_MAX_INTENSITY_MA;
    if (target < -MTQ_MAX_INTENSITY_MA) target = -MTQ_MAX_INTENSITY_MA;

    double q = round(target / MTQ_INTENSITY_STEP_MA) * MTQ_INTENSITY_STEP_MA;

    /* Update accumulator with the unrounded residual.  When raw is
     * sub-step, target eventually walks across the step boundary and
     * fires a single pulse; the accumulator then reverses sign and
     * the next pulse comes after the appropriate dead time. */
    *accumulator = target - q;

    return q;
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

    /* Step 3: 1st-order Σ-Δ dither: time-average sub-step commands */
    for (i = 0; i < 3; i++) {
        intensity_arr[i] = sigma_delta_step(intensity_arr[i],
                                            &sigma_delta_accumulator[i]);
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
