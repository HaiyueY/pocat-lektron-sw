/**
 * @file adcs_bench.h
 * @brief One-shot on-target timing of the ADCS reference models.
 *
 * Built only with -DADCS_BENCH=ON (./build.sh --adcs-bench). When enabled,
 * the ADCS task runs the benchmark once during setup and prints DWT cycle
 * counts and its stack high-water mark over the debug UART; see
 * docs/adcs_resource_budget.md for how the numbers are recorded.
 */

#ifndef ADCS_BENCH_H
#define ADCS_BENCH_H

/**
 * @brief Time SGP4, IGRF (several degrees), the Sun model and the full
 *        adcs_ephem_update() with the DWT cycle counter.
 *
 * No-op unless compiled with ADCS_BENCH. Must run in task context (it
 * kicks the ADCS health bit between measurements).
 */
void adcs_bench_run(void);

#endif /* ADCS_BENCH_H */
