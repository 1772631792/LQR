/* RTOS adapter sketch; the portable core itself has no RTOS dependency. */
#include "wheel_leg.h"
void WheelLegControlTask(void *argument) {
    (void)argument;wheel_leg_initialize();
    for(;;) {
        /* Wait for the exact periodic release, fill wheel_leg_U, call step. */
        wheel_leg_step();
        /* Send wheel_leg_Y through the project's motor driver. */
    }
}
