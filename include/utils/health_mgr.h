#include "health.h"


void health_init(void);

// period_ticks: obc hace check despues de period_ticks
// window_ticks: ventana de tiempo en la que cada tarea debe hacer kick
void health_config(TickType_t period, TickType_t window);

void health_set_expected(EventBits_t expected_bits);

int system_health(void);