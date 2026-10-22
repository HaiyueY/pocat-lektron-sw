/**
 * @file adcs_ctrl.h
 * @brief One ADCS control cycle: sensors, references, mode supervision, MTQ.
 *
 * Platform independent: no FreeRTOS or STM32 HAL. The ADCS task (adcs.c)
 * owns the timing, notifications and pause protocol and calls
 * adcs_ctrl_cycle() once per control period with the board I/O; a host
 * harness can call it with simulated I/O instead.
 *
 * Mode supervision: the OBC (or a TC) sets the *desired* mode; the
 * *current* mode (state.mode) follows it through IDLE:
 *
 *   desired IDLE        → IDLE
 *   desired DETUMBLING  → DETUMBLING until converged, then IDLE
 *                         (the desired mode also returns to IDLE)
 *   desired NADIR       → DETUMBLING first if |ω| > ADCS_NADIR_ENTRY_OMEGA,
 *                         then NADIR; back to DETUMBLING if
 *                         |ω| > ADCS_FDIR_NADIR_OMEGA for
 *                         ADCS_FDIR_OMEGA_COUNT cycles; IDLE while the
 *                         ephemeris is invalid or the TLE is stale
 *   any                 → IDLE after ADCS_SENSOR_FAIL_LIMIT consecutive
 *                         cycles without valid gyro and magnetometer
 *                         readings; retried once they recover
 *
 * Nadir does not actuate before its first sunlit TRIAD (adcs_nadir.c).
 */

#ifndef ADCS_CTRL_H
#define ADCS_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "adcs_types.h"
#include "adcs_ephemeris.h"

/* ---- adcs_ctrl_t.flags: status of the last cycle ---- */
#define ADCS_CTRL_MAG_FAIL      0x0001u  /**< Magnetometer read failed */
#define ADCS_CTRL_GYRO_FAIL     0x0002u  /**< Gyro read failed */
#define ADCS_CTRL_CSS_FAIL      0x0004u  /**< Sun-sensor read failed */
#define ADCS_CTRL_EPH_INVALID   0x0008u  /**< Ephemeris unusable (ADCS_EPH_INVALID_MASK) */
#define ADCS_CTRL_TLE_STALE     0x0010u  /**< TLE older than ADCS_TLE_MAX_AGE_S */
#define ADCS_CTRL_SENSOR_FAULT  0x0020u  /**< Held in IDLE by ADCS_SENSOR_FAIL_LIMIT */
#define ADCS_CTRL_FDIR_OMEGA    0x0040u  /**< Nadir left for detumbling this cycle */
#define ADCS_CTRL_MTQ_FAIL      0x0080u  /**< Magnetorquer enable/apply failed */

/** Board or simulated I/O used by one control cycle */
typedef struct {
    int (*read_mag)(mag_data_t *mag);
    int (*read_gyro)(gyro_data_t *gyro);
    int (*read_css)(sun_sensor_data_t *sun);
    int (*mtq_enable)(bool on);
    int (*mtq_apply)(const mtq_command_t *cmd);
} adcs_io_t;

/** ADCS controller context (statically allocated by the task) */
typedef struct {
    adcs_state_t state;         /**< Algorithm state; state.mode is the current mode */
    adcs_orbit_t orbit;         /**< TLE and SGP4 record */
    adcs_ephem_t eph;           /**< Last reference-model evaluation */
    adcs_mode_t  desired;       /**< Mode requested by the OBC */
    uint8_t      mtq_on;        /**< Magnetorquer drivers awake */
    uint8_t      sensor_fail_count;  /**< Consecutive cycles without gyro + mag */
    uint8_t      omega_high_count;   /**< Consecutive nadir cycles above the FDIR rate */
    uint16_t     flags;         /**< ADCS_CTRL_* bits of the last cycle */
    uint16_t     fdir_count;    /**< Nadir → detumbling transitions since init */
} adcs_ctrl_t;

/**
 * @brief Initialise the controller: IDLE, desired IDLE, no TLE.
 * @param[out] c  Controller context
 */
void adcs_ctrl_init(adcs_ctrl_t *c);

/**
 * @brief Set the desired mode.
 * @param[in,out] c     Controller context
 * @param[in]     mode  ADCS_MODE_IDLE, _DETUMBLING or _NADIR_POINTING
 * @return 0 on success, -1 for any other mode
 */
int adcs_ctrl_set_desired(adcs_ctrl_t *c, adcs_mode_t mode);

/**
 * @brief Load a new TLE (see adcs_orbit_load_tle(); the old one stays on failure).
 * @return ADCS_ORBIT_OK or a negative adcs_orbit_status_t code
 */
int adcs_ctrl_load_tle(adcs_ctrl_t *c, const char *line1, const char *line2);

/**
 * @brief Run one control cycle.
 *
 * Order: MTQ to zero (dead time) → gyro → sun sensors → magnetometer →
 * reference models → mode supervision → adcs_mode_step() → MTQ output.
 * The magnetometer is read last so the earlier reads cover the coil
 * current decay.
 *
 * @param[in,out] c       Controller context
 * @param[in]     io      Sensor and actuator I/O
 * @param[in]     unix_s  Current time [s since 1970, UTC]
 * @param[in]     dt      Time since the previous cycle's samples [s]
 * @return Period until the next cycle [s]
 */
double adcs_ctrl_cycle(adcs_ctrl_t *c, const adcs_io_t *io, double unix_s, double dt);

/**
 * @brief Put the magnetorquers to sleep immediately (pause, reset, fault).
 * @param[in,out] c   Controller context
 * @param[in]     io  Actuator I/O
 */
void adcs_ctrl_quiesce(adcs_ctrl_t *c, const adcs_io_t *io);

/**
 * @brief Restart after a pause: current mode IDLE, filters and counters cleared.
 *
 * The desired mode is kept; the next cycles sequence back into it.
 * @param[in,out] c  Controller context
 */
void adcs_ctrl_restart(adcs_ctrl_t *c);

#ifdef __cplusplus
}
#endif

#endif /* ADCS_CTRL_H */
