/* Adapter-only PID interface. The generated model never owns a PID algorithm. */
#ifndef WHEEL_LEG_PID_INTERFACE_H
#define WHEEL_LEG_PID_INTERFACE_H
#include "wlc.h"
#define WHEEL_LEG_PID_PORT_COUNT WLC_PID_PORT_COUNT
typedef WlcPidCalculateFn WheelLeg_PIDCalculateFn;
typedef WlcPidResetFn WheelLeg_PIDResetFn;
#endif
