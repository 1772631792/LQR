/* Call from a hardware timer at wheel_leg_P.core.sample_time_s. */
#include "wheel_leg.h"
void Control_Init(void) { wheel_leg_initialize(); }
void Control_1kHz(void) {
    /* TODO: write calibrated sensors and commands to wheel_leg_U. */
    wheel_leg_step();
    /* TODO: convert six wheel_leg_Y torque values to motor currents. */
    /* Check wheel_leg_Y.status_flags and apply the independent watchdog. */
}
