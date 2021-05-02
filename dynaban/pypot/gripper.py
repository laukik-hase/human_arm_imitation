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

def open_gripper():
    start = dxl_io.get_present_position([GRIPPER_ID])[0]
    cur = int(start)
    while(cur < OPEN_LIMIT):
        cur = cur + 1
        dxl_io.set_goal_position({GRIPPER_ID: cur})
        time.sleep(0.01)

    print("[+]Gripper opened")
    print(dxl_io.get_present_position([GRIPPER_ID]))

def close_gripper():
    start = dxl_io.get_present_position([GRIPPER_ID])[0]
    cur = int(start)
    while(cur > CLOSE_LIMIT):
        cur = cur - 1
        dxl_io.set_goal_position({GRIPPER_ID: cur})
        time.sleep(0.01)
    print("[+]Gripper closed")
    print(dxl_io.get_present_position([GRIPPER_ID]))

# close_gripper()
open_gripper()
time.sleep(1)
close_gripper()
time.sleep(1)
# open_gripper()
# time.sleep(1)

# dxl_io.set_goal_position({GRIPPER_ID: OPEN_LIMIT})
# time.sleep(1)
# dxl_io.set_pid_gain(DXL_DICT_PID)
# time.sleep(0.1)
