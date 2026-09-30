CC = gcc
PYTHON = python
CFLAGS = -std=c11 -O2 -Wall -Wextra -Werror -pedantic

.PHONY: all design test-c verify verify-gui verify-mujoco verify-production design-source run animate disturbance
all: controller/controller.dll controller/controller_test.exe controller/firmware.dll

controller/firmware.dll: controller/firmware_bridge.c controller/riccati6.c balance_chassis-main/application/chassis/lqr_calc.h balance_chassis-main/application/chassis/linkNleg.h balance_chassis-main/application/chassis/balance.h
	$(CC) $(CFLAGS) -shared -Icontroller/firmware_compat -Ibalance_chassis-main/application/chassis controller/firmware_bridge.c controller/riccati6.c -o $@

design:
	$(PYTHON) -m simulation.design_lqr

controller/lqr_gains.h: config/robot_params.py simulation/design_lqr.py simulation/model.py
	$(PYTHON) -m simulation.design_lqr

controller/controller.dll: controller/lqr.c controller/lqr.h controller/lqr_gains.h controller/lqr_design.c controller/lqr_design.h controller/chassis.c controller/chassis.h
	$(CC) $(CFLAGS) -shared -DLQR_BUILD_DLL controller/lqr.c controller/lqr_design.c controller/chassis.c -o $@

controller/controller_test.exe: controller/main.c controller/lqr.c controller/lqr.h controller/lqr_gains.h
	$(CC) $(CFLAGS) controller/main.c controller/lqr.c -o $@

test-c: all
	./controller/controller_test.exe

verify: test-c
	$(PYTHON) -m simulation.verify
	$(PYTHON) -m simulation.five_bar
	$(PYTHON) -m simulation.verify_chassis

verify-gui: all
	$(PYTHON) -m simulation.verify_host

verify-mujoco: all
	$(PYTHON) -m simulation.verify_mujoco
	$(PYTHON) -m simulation.verify_mujoco_gui

verify-production: all
	$(PYTHON) -m simulation.verify_production_export

design-source: all
	$(PYTHON) -m simulation.source_design

run: all
	$(PYTHON) main.py

animate: all
	$(PYTHON) main.py

disturbance: all
	$(PYTHON) main.py --impulse 0.5 --impulse-time 3 --output outputs/disturbance
