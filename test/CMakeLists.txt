cmake_minimum_required(VERSION 3.12)
project(adcs_test C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -g -O0")

# ADCS source directories (new pocat-lektron-sw layout)
set(ADCS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src/subsystems/adcs)
set(ADCS_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include/subsystems/adcs)

# Include paths: ADCS headers + HAL stubs + simulators
include_directories(
    ${ADCS_INC_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/hal_stub
    ${CMAKE_CURRENT_SOURCE_DIR}/sim
)

# ADCS algorithm sources (platform-independent)
set(ADCS_SOURCES
    ${ADCS_SRC_DIR}/adcs_magnetorquer.c
    ${ADCS_SRC_DIR}/adcs_detumble.c
    ${ADCS_SRC_DIR}/adcs_determination.c
    ${ADCS_SRC_DIR}/adcs_nadir.c
    ${ADCS_SRC_DIR}/adcs_mode_manager.c
    ${ADCS_SRC_DIR}/adcs_time.c
    ${ADCS_SRC_DIR}/adcs_tle.c
    ${ADCS_SRC_DIR}/adcs_sgp4.c
    ${ADCS_SRC_DIR}/adcs_igrf.c
    ${ADCS_SRC_DIR}/adcs_sun.c
    ${ADCS_SRC_DIR}/adcs_ephemeris.c
)

# Simulator sources
set(SIM_SOURCES
    sim/sim_environment.c
    sim/sim_magnetometer.c
    sim/sim_gyroscope.c
    sim/sim_photodiode.c
    sim/sim_mtq_driver.c
)

# Link math library
link_libraries(m)

# --- Test executables ---

add_executable(test_detumble
    test_detumble.c
    ${ADCS_SOURCES}
    ${SIM_SOURCES}
)

add_executable(test_nadir
    test_nadir.c
    ${ADCS_SOURCES}
    ${SIM_SOURCES}
)

add_executable(test_mode_manager
    test_mode_manager.c
    ${ADCS_SOURCES}
    ${SIM_SOURCES}
)

add_executable(test_freq_sweep
    test_freq_sweep.c
    ${ADCS_SOURCES}
    ${SIM_SOURCES}
)

add_executable(test_monte_carlo
    test_monte_carlo.c
    ${ADCS_SOURCES}
    ${SIM_SOURCES}
)

add_executable(test_ephemeris
    test_ephemeris.c
    ${ADCS_SOURCES}
)

add_executable(test_igrf_order
    test_igrf_order.c
    ${ADCS_SOURCES}
)

# --- Regression tests (ctest) ---
# test_freq_sweep and test_igrf_order are study tools, not pass/fail tests.

enable_testing()
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/results)

add_test(NAME mode_manager      COMMAND test_mode_manager)
add_test(NAME detumble          COMMAND test_detumble)
add_test(NAME nadir             COMMAND test_nadir)
add_test(NAME monte_carlo_30dps COMMAND test_monte_carlo 30 50 --seed 42)
add_test(NAME monte_carlo_90dps COMMAND test_monte_carlo 90 50 --seed 42)
add_test(NAME ephemeris         COMMAND test_ephemeris ${CMAKE_CURRENT_SOURCE_DIR}/data)
add_test(NAME nadir_ephem       COMMAND test_nadir --ephem)

set_tests_properties(mode_manager detumble nadir monte_carlo_30dps monte_carlo_90dps ephemeris
                     nadir_ephem
    PROPERTIES
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        ENVIRONMENT ADCS_TEST_NO_PLOT=1
        TIMEOUT 120
)
