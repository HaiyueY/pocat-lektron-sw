/**
 * @file css.c
 * @brief Coarse sun sensor sampling through the TMUX1108.
 */

#include "drivers/css.h"
#include "drivers/adcs_bus.h"

/* Mux channel (0 = S1) of each body face, order +X, −X, +Y, −Y, +Z, −Z */
static const uint8_t mux_of_face[CSS_CHANNELS] = { 1, 0, 5, 4, 3, 2 };

/* S7 is tied to GND: the ADC zero reference */
#define MUX_ZERO    6u

static int sample_channel(uint8_t channel, double *volts)
{
    double sum = 0.0;
    double v;
    int i;

    adcs_bus_css_select(channel);
    adcs_bus_delay_us(CSS_SETTLE_US);
    for (i = 0; i < CSS_SAMPLES; i++) {
        if (adcs_bus_css_sample(&v) != 0) {
            return -1;
        }
        sum += v;
    }
    *volts = sum / CSS_SAMPLES;
    return 0;
}

int css_read(double volts[CSS_CHANNELS])
{
    double zero;
    int i;

    if (sample_channel(MUX_ZERO, &zero) != 0) {
        return -1;
    }
    for (i = 0; i < CSS_CHANNELS; i++) {
        if (sample_channel(mux_of_face[i], &volts[i]) != 0) {
            return -1;
        }
        volts[i] -= zero;
    }
    return 0;
}
