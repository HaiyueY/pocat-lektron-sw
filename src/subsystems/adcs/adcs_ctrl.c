/**
 * @file adcs_ctrl.c
 * @brief One ADCS control cycle: sensors, references, mode supervision, MTQ.
 *
 * See adcs_ctrl.h for the supervision rules. Ephemeris evaluation is
 * skipped while detumbling (the ω×B law needs no references), which saves
 * the ≈ 0.2 M-cycle soft-float update at low clock rates.
 */

#include "adcs_ctrl.h"
#include "adcs_config.h"
#include "adcs_determination.h"
#include "adcs_detumble.h"
#include "adcs_math.h"
#include "adcs_mode_manager.h"
#include <string.h>

/** Zero magnetorquer command, applied at the start of every cycle */
static const mtq_command_t mtq_zero;

static void go_idle(adcs_ctrl_t *c)
{
    if (c->state.mode != ADCS_MODE_IDLE) {
        (void)adcs_mode_request(&c->state, ADCS_MODE_IDLE);
    }
    c->omega_high_count = 0;
}

static void enter(adcs_ctrl_t *c, adcs_mode_t mode)
{
    go_idle(c);
    (void)adcs_mode_request(&c->state, mode);
}

/** Read the sensors; the magnetometer last (see adcs_ctrl_cycle()). */
static void read_sensors(adcs_ctrl_t *c, const adcs_io_t *io)
{
    adcs_state_t *s = &c->state;

    if (io->read_gyro(&s->gyro) != 0) {
        c->flags |= ADCS_CTRL_GYRO_FAIL;
    }
    if (io->read_css(&s->sun) != 0) {
        c->flags |= ADCS_CTRL_CSS_FAIL;
    }
    if (io->read_mag(&s->mag) != 0) {
        c->flags |= ADCS_CTRL_MAG_FAIL;
    }
    determination_css_sun(&s->sun);
}

/**
 * @brief Evaluate the reference models and copy them into the state.
 * @return 1 if the references are usable for nadir pointing
 */
static int update_references(adcs_ctrl_t *c, double unix_s)
{
    uint8_t status = adcs_ephem_update(&c->orbit, unix_s, ADCS_IGRF_ORDER, &c->eph);

    if (status & ADCS_EPH_TLE_STALE) {
        c->flags |= ADCS_CTRL_TLE_STALE;
    }
    if (adcs_ephem_to_state(&c->eph, &c->state) != 0) {
        c->flags |= ADCS_CTRL_EPH_INVALID;
        return 0;
    }

    /* The CSS can be fooled by Earth albedo; trust them only outside the
     * modelled shadow */
    if (c->eph.eclipse) {
        c->state.sun.sun_visible = 0;
    }
    return (status & ADCS_EPH_TLE_STALE) ? 0 : 1;
}

/** Move the current mode towards the desired one (adcs_ctrl.h). */
static void supervise(adcs_ctrl_t *c, int refs_ok)
{
    adcs_state_t *s = &c->state;
    int sensors_ok = s->gyro.valid && s->mag.valid;

    if (sensors_ok) {
        c->sensor_fail_count = 0;
    } else if (c->sensor_fail_count < 255u) {
        c->sensor_fail_count++;
    }
    if (c->sensor_fail_count >= ADCS_SENSOR_FAIL_LIMIT) {
        c->flags |= ADCS_CTRL_SENSOR_FAULT;
        go_idle(c);
        return;
    }

    switch (c->desired) {
    case ADCS_MODE_DETUMBLING:
        if (s->mode != ADCS_MODE_DETUMBLING && sensors_ok) {
            enter(c, ADCS_MODE_DETUMBLING);
        }
        break;

    case ADCS_MODE_NADIR_POINTING:
        if (s->mode == ADCS_MODE_NADIR_POINTING) {
            if (!refs_ok) {
                go_idle(c);
                break;
            }
            /* Raw rate: the bias estimate is ≈ 0.01 °/s, and must not be
             * trusted to tell a real spin-up from a bias */
            if (s->gyro.valid &&
                vec3d_norm(s->gyro.angular_vel) > ADCS_FDIR_NADIR_OMEGA) {
                c->omega_high_count++;
            } else {
                c->omega_high_count = 0;
            }
            if (c->omega_high_count >= ADCS_FDIR_OMEGA_COUNT) {
                c->flags |= ADCS_CTRL_FDIR_OMEGA;
                c->fdir_count++;
                enter(c, ADCS_MODE_DETUMBLING);
            }
        } else if (s->mode == ADCS_MODE_IDLE && sensors_ok) {
            /* Detumbling needs no references, so a high rate is damped
             * even while the ephemeris is unusable */
            if (vec3d_norm(s->gyro.angular_vel) > ADCS_NADIR_ENTRY_OMEGA) {
                enter(c, ADCS_MODE_DETUMBLING);
            } else if (refs_ok) {
                enter(c, ADCS_MODE_NADIR_POINTING);
            }
        }
        /* DETUMBLING on the way to NADIR: wait for convergence */
        break;

    case ADCS_MODE_IDLE:
    default:
        go_idle(c);
        break;
    }
}

/** Drive the magnetorquers for the current mode. */
static void apply_output(adcs_ctrl_t *c, const adcs_io_t *io)
{
    adcs_mode_t mode = c->state.mode;

    if (mode == ADCS_MODE_DETUMBLING || mode == ADCS_MODE_NADIR_POINTING) {
        if (!c->mtq_on) {
            if (io->mtq_enable(true) != 0) {
                c->flags |= ADCS_CTRL_MTQ_FAIL;
            }
            c->mtq_on = 1;
        }
        if (io->mtq_apply(&c->state.mtq_cmd) != 0) {
            c->flags |= ADCS_CTRL_MTQ_FAIL;
        }
    } else if (c->mtq_on) {
        (void)io->mtq_enable(false);
        c->mtq_on = 0;
    }
}

void adcs_ctrl_init(adcs_ctrl_t *c)
{
    memset(c, 0, sizeof(*c));
    adcs_mode_init(&c->state);
    adcs_orbit_init(&c->orbit);
    c->eph.status = ADCS_EPH_NO_TLE;
    c->desired = ADCS_MODE_IDLE;
}

int adcs_ctrl_set_desired(adcs_ctrl_t *c, adcs_mode_t mode)
{
    if (mode != ADCS_MODE_IDLE && mode != ADCS_MODE_DETUMBLING &&
        mode != ADCS_MODE_NADIR_POINTING) {
        return -1;
    }
    c->desired = mode;
    return 0;
}

int adcs_ctrl_load_tle(adcs_ctrl_t *c, const char *line1, const char *line2)
{
    return adcs_orbit_load_tle(&c->orbit, line1, line2);
}

double adcs_ctrl_cycle(adcs_ctrl_t *c, const adcs_io_t *io, double unix_s, double dt)
{
    adcs_state_t *s = &c->state;
    int refs_ok = 0;

    c->flags = 0;

    /* Dead time: no torque while the field is measured */
    if (c->mtq_on && io->mtq_apply(&mtq_zero) != 0) {
        c->flags |= ADCS_CTRL_MTQ_FAIL;
    }

    read_sensors(c, io);

    if (s->mode != ADCS_MODE_DETUMBLING) {
        refs_ok = update_references(c, unix_s);
    }

    supervise(c, refs_ok);

    s->dt = dt;
    int converged = (adcs_mode_step(s) == 1);
    if (converged && c->desired == ADCS_MODE_DETUMBLING) {
        c->desired = ADCS_MODE_IDLE;
    }

    apply_output(c, io);

    switch (s->mode) {
    case ADCS_MODE_DETUMBLING:
        return detumble_select_dt(s);
    case ADCS_MODE_NADIR_POINTING:
        return ADCS_CONTROL_DT;
    default:
        /* Just converged on the way to NADIR: enter it next period */
        return (converged && c->desired != ADCS_MODE_IDLE) ? ADCS_CONTROL_DT
                                                           : ADCS_IDLE_PERIOD_S;
    }
}

void adcs_ctrl_quiesce(adcs_ctrl_t *c, const adcs_io_t *io)
{
    (void)io->mtq_enable(false);
    c->mtq_on = 0;
    c->state.mtq_cmd = mtq_zero;
}

void adcs_ctrl_restart(adcs_ctrl_t *c)
{
    go_idle(c);
    c->state.mtq_cmd = mtq_zero;
    c->sensor_fail_count = 0;
    c->omega_high_count = 0;
}
