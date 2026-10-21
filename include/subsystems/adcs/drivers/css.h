/**
 * @file css.h
 * @brief Coarse sun sensors: six photodiode TIAs behind a TMUX1108 into one ADC pin.
 *
 * Each face photodiode (SLCD-61N8 on the face boards) has its own LPV542
 * transimpedance stage (2 kΩ ‖ 100 pF) on PQ_ADCS_HBridge; the TMUX1108
 * selects one TIA output onto ADC_PH. Mux inputs:
 *
 *   S1 −X   S2 +X   S3 −Z   S4 +Z   S5 −Y   S6 +Y (PH_PL)   S7, S8 GND
 *
 * css_read() returns the voltages in body-face order +X, −X, +Y, −Y, +Z,
 * −Z (as sun_sensor_data_t), with the GND channel subtracted as the ADC
 * zero.
 */

#ifndef CSS_H
#define CSS_H

#ifdef __cplusplus
extern "C" {
#endif

/** Number of sun-sensor channels */
#define CSS_CHANNELS        6

/** ADC samples averaged per channel */
#define CSS_SAMPLES         4

/** Settling after a mux switch before sampling [µs] */
#define CSS_SETTLE_US       20u

/**
 * @brief Sample all six sun sensors.
 * @param[out] volts  TIA output voltages, body-face order [V]
 * @return 0 on success, -1 on ADC error
 */
int css_read(double volts[CSS_CHANNELS]);

#ifdef __cplusplus
}
#endif

#endif /* CSS_H */
