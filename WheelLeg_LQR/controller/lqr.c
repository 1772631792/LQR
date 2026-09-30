#include "lqr.h"
#include "lqr_gains.h"
#include <math.h>
#include <stddef.h>

void LQR_Init(LQRController *ctrl)
{
    static const float gains[4] = LQR_DEFAULT_K;
    unsigned int i;
    if (ctrl == NULL) { return; }
    for (i = 0; i < 4; ++i) { ctrl->K[i] = gains[i]; }
    ctrl->output_limit = LQR_OUTPUT_LIMIT;
}

float LQR_Update(LQRController *ctrl, const RobotState *state,
                 const RobotState *reference)
{
    float force;
    if (ctrl == NULL || state == NULL || reference == NULL) { return 0.0f; }
    if (!isfinite(ctrl->output_limit) || ctrl->output_limit <= 0.0f) { return 0.0f; }
    force = -(ctrl->K[0] * (state->pos - reference->pos)
            + ctrl->K[1] * (state->vel - reference->vel)
            + ctrl->K[2] * (state->pitch - reference->pitch)
            + ctrl->K[3] * (state->pitch_rate - reference->pitch_rate));
    /* Simulation invalid-input policy: return zero. A real MCU needs fault handling. */
    if (!isfinite(force)) { return 0.0f; }
    if (force > ctrl->output_limit) { force = ctrl->output_limit; }
    if (force < -ctrl->output_limit) { force = -ctrl->output_limit; }
    return force;
}

float LQR_GetSampleTime(void) { return LQR_SAMPLE_TIME; }
const char *LQR_GetDesignId(void) { return LQR_DESIGN_ID; }
unsigned int LQR_GetStateSize(void) { return (unsigned int)sizeof(RobotState); }
unsigned int LQR_GetControllerSize(void) { return (unsigned int)sizeof(LQRController); }
