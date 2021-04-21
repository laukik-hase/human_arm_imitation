import time
import json

# pypot imports
import pypot.robot
import pypot.dynamixel
# import matplotlib.pyplot as plt
# import manipulator_math_utils as mmu
import sys
import csv

import pprint
pp = pprint.PrettyPrinter(indent=4)

SPLINE = 10000
motor_id = [1]
JOINTS = len(motor_id)
DXL_DICT_3      = dict(zip(motor_id, [3]*JOINTS))
DXL_DICT_1      = dict(zip(motor_id, [1]*JOINTS))
DXL_DICT_0      = dict(zip(motor_id, [0]*JOINTS))
DXL_DICT_PID      = dict(zip(motor_id, [[16,0,0]]*JOINTS))


ports = pypot.dynamixel.get_available_ports()

if not ports:
    raise IOError('no port found!')

print('ports found', ports)

print('connecting on the first available port:', ports[0])

dxl_io = pypot.dynamixel.DxlIO(ports[0],baudrate = 57600)

def setTraj1(id, duration, coeffs):
    errorCounter = 0
    delay = 0.001
    while True:
        try:
            dxl_io.set_traj1_size({id: 4})
            time.sleep(delay)
            dxl_io.set_duration1({id: duration})
            time.sleep(delay)
            dxl_io.set_a0_traj1({id: coeffs[0]})
            time.sleep(delay)
            dxl_io.set_a1_traj1({id: coeffs[1]})
            time.sleep(delay)
            dxl_io.set_a2_traj1({id: coeffs[2]})
            time.sleep(delay)
            dxl_io.set_a3_traj1({id: coeffs[3]})
            time.sleep(delay)
#             dxl_io.set_a4_traj1({id: coeffs[4]})
#             time.sleep(delay)

            break
        except:
            errorCounter = errorCounter + 1
            # print "Nope :/"
            break
            print("nb errors1 = ", errorCounter)


def setTraj2(id, duration, coeffs):
    errorCounter = 0
    delay = 0.001

    while True:
        try:
            dxl_io.set_traj2_size({id: 4})
            time.sleep(delay)
            dxl_io.set_duration2({id: duration})
            time.sleep(delay)
            dxl_io.set_a0_traj2({id: coeffs[0]})
            time.sleep(delay)
            dxl_io.set_a1_traj2({id: coeffs[1]})
            time.sleep(delay)
            dxl_io.set_a2_traj2({id: coeffs[2]})
            time.sleep(delay)
            dxl_io.set_a3_traj2({id: coeffs[3]})
            time.sleep(delay)
#             dxl_io.set_a4_traj2({id: coeffs[4]})
#             time.sleep(delay)

            break
        except:
            errorCounter = errorCounter + 1
            print("nb errors2 = ", errorCounter)
            break


def setTorque1(id, duration, coeffs):
    errorCounter = 0
    delay = 0.001
    while True:
        try:
            dxl_io.set_torque1_size({id: 4})
            time.sleep(delay)
            dxl_io.set_duration1({id: duration})
            time.sleep(delay)
            dxl_io.set_a0_torque1({id: coeffs[0]})
            time.sleep(delay)
            dxl_io.set_a1_torque1({id: coeffs[1]})
            time.sleep(delay)
            dxl_io.set_a2_torque1({id: coeffs[2]})
            time.sleep(delay)
            dxl_io.set_a3_torque1({id: coeffs[3]})
            time.sleep(delay)
#             dxl_io.set_a4_torque1({id: coeffs[4]})
#             time.sleep(delay)
            break
        except:
            errorCounter = errorCounter + 1
            # print "Nope :/"
            pass
#         print "Nb errors : ", errorCounter

def setTorque2(id, duration, coeffs):
    errorCounter = 0
    delay = 0.001
    while True:
        try:
            dxl_io.set_torque2_size({id: 4})
            time.sleep(delay)
            dxl_io.set_duration2({id: duration})
            time.sleep(delay)
            dxl_io.set_a0_torque2({id: coeffs[0]})
            time.sleep(delay)
            dxl_io.set_a1_torque2({id: coeffs[1]})
            time.sleep(delay)
            dxl_io.set_a2_torque2({id: coeffs[2]})
            time.sleep(delay)
            dxl_io.set_a3_torque2({id: coeffs[3]})
            time.sleep(delay)
#             dxl_io.set_a4_torque2({id: coeffs[4]})
#             time.sleep(delay)
            break
        except:
            errorCounter = errorCounter + 1
            # print "Nope :/"
            pass
ID = 1
# dxl_io.factory_reset([ID], except_baudrate_and_ids=True)
dxl_io.set_mode_dynaban({ID: 0})
time.sleep(0.1)
dxl_io.enable_torque({ID: 1})
time.sleep(0.1)
dxl_io.set_goal_position({ID: 0})
time.sleep(1)
dxl_io.set_pid_gain({ID: [1, 0, 0]})
time.sleep(0.1)
setTraj1(motor_id[0], SPLINE, [2048.0, 1024.0, 0.0, 0.0])
dxl_io.set_mode_dynaban(DXL_DICT_3) 
time.sleep(2)
# dxl_io.set_mode_dynaban(DXL_DICT_0) 
