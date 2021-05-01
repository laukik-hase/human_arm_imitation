import time
import numpy
import json

# pypot imports
import pypot.dynamixel

ports = pypot.dynamixel.get_available_ports()

if not ports:
    raise IOError('no port found!')

print('ports found', ports)

print('connecting on the first available port:', ports[0])
dxl_io = pypot.dynamixel.DxlIO(ports[0])

GRIPPER_ID = 7

OPEN_LIMIT = 150 # in degrees for pypot, 1023 in steps (Max angle for AX-12A)
CLOSE_LIMIT = 100 #81.96 # in degrees for pypot, 791 in steps


dxl_io.enable_torque({GRIPPER_ID: 1})
time.sleep(0.1)

dxl_io.set_goal_position({GRIPPER_ID: OPEN_LIMIT})
time.sleep(1)
print(dxl_io.get_present_position([GRIPPER_ID]))
dxl_io.set_goal_position({GRIPPER_ID: CLOSE_LIMIT})
time.sleep(1)
print(dxl_io.get_present_position([GRIPPER_ID]))
# dxl_io.set_goal_position({GRIPPER_ID: OPEN_LIMIT})
# time.sleep(1)
# dxl_io.set_pid_gain(DXL_DICT_PID)
# time.sleep(0.1)
