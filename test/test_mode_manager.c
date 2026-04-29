/**
 * @file test_mode_manager.c
 * @brief Mode state machine transition test.
 *
 * Verifies all valid and invalid mode transitions:
 *   IDLE → DETUMBLING → IDLE → NADIR_POINTING → IDLE → SAFE
 *   Invalid: SAFE → IDLE, DETUMBLING → NADIR_POINTING
 */

#include <stdio.h>
#include "adcs_types.h"
#include "adcs_config.h"
#include "adcs_math.h"
#include "adcs_mode_manager.h"

static const char *mode_name(adcs_mode_t m)
{
    switch (m) {
    case ADCS_MODE_IDLE:            return "IDLE";
    case ADCS_MODE_DETUMBLING:      return "DETUMBLING";
    case ADCS_MODE_NADIR_POINTING:  return "NADIR_POINTING";
    case ADCS_MODE_SAFE:            return "SAFE";
    default:                        return "UNKNOWN";
    }
}

static int test_count = 0;
static int pass_count = 0;

static void check(const char *desc, int condition)
{
    test_count++;
    if (condition) {
        printf("  PASS: %s\n", desc);
        pass_count++;
    } else {
        printf("  FAIL: %s\n", desc);
    }
}

int main(void)
{
    printf("=== ADCS Mode Manager Test ===\n\n");

    adcs_state_t state;
    adcs_mode_init(&state);

    /* Test 1: Initial state is IDLE */
    printf("[Test 1] Initial state\n");
    check("Initial mode is IDLE", state.mode == ADCS_MODE_IDLE);

    /* Test 2: IDLE → DETUMBLING */
    printf("\n[Test 2] IDLE → DETUMBLING\n");
    int rc = adcs_mode_request(&state, ADCS_MODE_DETUMBLING);
    check("Transition accepted (rc=0)", rc == 0);
    check("Mode is DETUMBLING", state.mode == ADCS_MODE_DETUMBLING);

    /* Test 3: DETUMBLING → NADIR_POINTING (invalid) */
    printf("\n[Test 3] DETUMBLING → NADIR_POINTING (invalid)\n");
    rc = adcs_mode_request(&state, ADCS_MODE_NADIR_POINTING);
    check("Transition rejected (rc=-1)", rc == -1);
    check("Mode remains DETUMBLING", state.mode == ADCS_MODE_DETUMBLING);

    /* Test 4: DETUMBLING → IDLE */
    printf("\n[Test 4] DETUMBLING → IDLE\n");
    rc = adcs_mode_request(&state, ADCS_MODE_IDLE);
    check("Transition accepted (rc=0)", rc == 0);
    check("Mode is IDLE", state.mode == ADCS_MODE_IDLE);

    /* Test 5: IDLE → NADIR_POINTING */
    printf("\n[Test 5] IDLE → NADIR_POINTING\n");
    rc = adcs_mode_request(&state, ADCS_MODE_NADIR_POINTING);
    check("Transition accepted (rc=0)", rc == 0);
    check("Mode is NADIR_POINTING", state.mode == ADCS_MODE_NADIR_POINTING);

    /* Test 6: NADIR_POINTING → SAFE */
    printf("\n[Test 6] NADIR_POINTING → SAFE\n");
    rc = adcs_mode_request(&state, ADCS_MODE_SAFE);
    check("Transition accepted (rc=0)", rc == 0);
    check("Mode is SAFE", state.mode == ADCS_MODE_SAFE);
    check("Dipole zeroed", state.mtq_cmd.dipole.x == 0.0 &&
                           state.mtq_cmd.dipole.y == 0.0 &&
                           state.mtq_cmd.dipole.z == 0.0);

    /* Test 7: SAFE → IDLE (invalid) */
    printf("\n[Test 7] SAFE → IDLE (invalid)\n");
    rc = adcs_mode_request(&state, ADCS_MODE_IDLE);
    check("Transition rejected (rc=-1)", rc == -1);
    check("Mode remains SAFE", state.mode == ADCS_MODE_SAFE);

    /* Test 8: IDLE step zeroes actuators */
    printf("\n[Test 8] IDLE mode step\n");
    state.mode = ADCS_MODE_IDLE;
    state.mtq_cmd.dipole.x = 1.0;
    adcs_mode_step(&state);
    check("Dipole zeroed in IDLE step",
          state.mtq_cmd.dipole.x == 0.0 &&
          state.mtq_cmd.dipole.y == 0.0 &&
          state.mtq_cmd.dipole.z == 0.0);

    /* Test 9: Auto-transition from DETUMBLING when ω converges */
    printf("\n[Test 9] Auto-transition on detumble convergence\n");
    adcs_mode_request(&state, ADCS_MODE_DETUMBLING);
    /* Simulate converged state: ω well below threshold */
    state.gyro.angular_vel = vec3d_make(0.001, 0.001, 0.001);
    state.mag.field = vec3d_make(3e-5, 1e-5, 2e-5);
    state.mag_field_prev = vec3d_make(3e-5, 1e-5, 2e-5);
    state.dt = ADCS_CONTROL_DT;
    /* Run enough steps to trigger stable count */
    int auto_done = 0;
    for (int i = 0; i < DETUMBLE_STABLE_COUNT + 5; i++) {
        int r = adcs_mode_step(&state);
        if (r == 1) {
            auto_done = 1;
            break;
        }
    }
    check("Auto-transition to IDLE on convergence", auto_done);
    check("Mode is IDLE after convergence", state.mode == ADCS_MODE_IDLE);

    /* Summary */
    printf("\n=== Results: %d/%d tests passed ===\n", pass_count, test_count);

    if (pass_count == test_count) {
        printf("PASS: All mode manager tests passed\n");
        return 0;
    } else {
        printf("FAIL: %d tests failed\n", test_count - pass_count);
        return 1;
    }
}
