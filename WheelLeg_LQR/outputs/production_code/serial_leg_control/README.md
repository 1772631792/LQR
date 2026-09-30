# Wheel-leg generated control model

The public layout follows MATLAB/Simulink Coder conventions: write `wheel_leg_U`,
call `wheel_leg_step()`, then read `wheel_leg_Y`. Intermediate block signals are
in `wheel_leg_B`, discrete/PID states in `wheel_leg_DW`, and tunable parameters
in `wheel_leg_P`.

The generated model does not implement or own a PID algorithm. Bind the application's
existing PID structure and calculate function through `wheel_leg_bind_pid()`, then
call it from `src/wheel_leg_user.c`. Thus advanced filtering, DWT timing, anti-windup
and error handling remain in the application's own PID implementation. User files
are preserved when exporting again.

Integration contract:

1. Call `wheel_leg_initialize()` once.
2. Calibrate sensors into `wheel_leg_U` using SI units and documented signs.
3. Call `wheel_leg_step()` exactly once per configured sample period.
4. Convert `wheel_leg_Y` torque in Nm to motor current in the hardware adapter.
5. Independently implement watchdog, physical emergency stop and board protection.

Build the independent self-test:

```sh
gcc -std=c11 -O2 -Wall -Wextra -Werror -pedantic -Iinclude -Igenerated src/*.c tests/test_runner.c -lm -o wheel_leg_test
./wheel_leg_test
```

The package deliberately contains no HAL, RTOS, CAN, MuJoCo or Python dependency.
Passing the self-test does not authorize high-power operation; follow staged motor,
single-leg, restraint-frame and low-torque commissioning.
