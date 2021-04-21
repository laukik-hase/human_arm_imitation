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
dxl_io = pypot.dynamixel.DxlIO(ports[0], baudrate=57600)

ID_LIST = [2,3,1]
JOINTS = len(ID_LIST)

DXL_DICT_1      = dict(zip(ID_LIST, [1]*JOINTS))
DXL_DICT_0      = dict(zip(ID_LIST, [0]*JOINTS))
DXL_DICT_PID    = dict(zip(ID_LIST, [[1,0,0]]*JOINTS))

cur_pos = [0,90,0]
DXL_DICT_cur_pos = dict(zip(ID_LIST, cur_pos))

dxl_io.set_mode_dynaban(DXL_DICT_0)
time.sleep(0.1)
dxl_io.enable_torque(DXL_DICT_1)
time.sleep(0.1)
dxl_io.set_goal_position(DXL_DICT_cur_pos)
time.sleep(1)
dxl_io.set_pid_gain(DXL_DICT_PID)
time.sleep(0.1)
