#!/usr/bin/env python
import arm_control_utils

DURATION                    = 30000
TRAJ_POLY1                  = [1000, 100, 100]
TORQUE_POLY1                = [1000, 100, 100]
MODE                        = 3

arm_control_utils.initialize_motors()
arm_control_utils.enable_state_torque()
arm_control_utils.set_debug(1, 0)

print("Ready to move")

arm_control_utils.set_position_trajectory(1, DURATION, TRAJ_POLY1, TORQUE_POLY1)
arm_control_utils.set_mode(1, MODE)

arm_control_utils.disable_state_torque()
arm_control_utils.stop_motors()