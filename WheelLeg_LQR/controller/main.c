/* Standalone C verification. MCU port: call LQR_Update once per 1 ms timer tick. */
#include "lqr.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    static LQRController ctrl;
    static RobotState state = {0.0f, 0.0f, 0.0f, 0.0f};
    static RobotState reference = {0.0f, 0.0f, 0.0f, 0.0f};
    LQR_Init(&ctrl);
    assert(fabsf(LQR_GetSampleTime() - 0.001f) < 1e-9f);
    assert(LQR_Update(&ctrl, &state, &reference) == 0.0f);
    state.pitch = 0.174532925f;
    assert(LQR_Update(&ctrl, &state, &reference) > 0.0f);
    state.pitch = 100.0f;
    assert(LQR_Update(&ctrl, &state, &reference) == ctrl.output_limit);
    state.pitch = -100.0f;
    assert(LQR_Update(&ctrl, &state, &reference) == -ctrl.output_limit);
    reference = state;
    assert(LQR_Update(&ctrl, &state, &reference) == 0.0f);
    state.pitch = NAN;
    assert(LQR_Update(&ctrl, &state, &reference) == 0.0f);
    assert(LQR_Update(NULL, &state, &reference) == 0.0f);
    printf("PASS: C equilibrium, sign, both saturation limits, reference, invalid input. Ts=%.6f s\n",
           (double)LQR_GetSampleTime());
    return 0;
}
