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

ID_LIST = [1, 2, 3, 4]
ID_SIZE = len(ID_LIST)
print("Test with PID only:")

cur_pos = [90, 0, 0, 0]
for i in range(ID_SIZE):
    dxl_io.set_mode_dynaban({ID_LIST[i]: 0})
time.sleep(0.1)
for i in range(ID_SIZE):
    dxl_io.enable_torque({ID_LIST[i]: 1})
time.sleep(0.1)
for i in range(ID_SIZE):
    dxl_io.set_goal_position({ID_LIST[i]: cur_pos[i]})
time.sleep(1)
for i in range(ID_SIZE):
    dxl_io.set_pid_gain({ID_LIST[i]: [1, 0, 0]})
time.sleep(0.1)
