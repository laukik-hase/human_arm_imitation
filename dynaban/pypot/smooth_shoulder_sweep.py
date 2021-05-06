import time
import numpy
import json

# pypot imports
import pypot.dynamixel
import math
import matplotlib.pyplot as plt
import numpy as np 
from scipy.optimize import curve_fit
import csv
import pypot.dynamixel
import sys

state_file = open("Shoulder_sweep_with_torque.csv", "w")

num1 = 1
num2 = 1


ports = pypot.dynamixel.get_available_ports()

if not ports:
    raise IOError('no port found!')

print('ports found', ports)

print('connecting on the first available port:', ports[0])
dxl_io = pypot.dynamixel.DxlIO(ports[0])


def setTraj1(id, duration, coeffs):
    errorCounter = 0
    delay = 0.001
    while True:
        try:
            dxl_io.set_traj1_size({id: 3})
            time.sleep(delay)
            dxl_io.set_duration1({id: duration})
            time.sleep(delay)
            dxl_io.set_a0_traj1({id: coeffs[0]})
            time.sleep(delay)
            dxl_io.set_a1_traj1({id: coeffs[1]})
            time.sleep(delay)
            dxl_io.set_a2_traj1({id: coeffs[2]})
            time.sleep(delay)
            break
        except:
            errorCounter = errorCounter + 1
            # print "Nope :/"
            break
#         print "Nb errors : ", errorCounter


def setTraj2(id, duration, coeffs):
    errorCounter = 0
    delay = 0.001

    while True:
        try:
            dxl_io.set_traj2_size({id: 3})
            time.sleep(delay)
            dxl_io.set_duration2({id: duration})
            time.sleep(delay)
            dxl_io.set_a0_traj2({id: coeffs[0]})
            time.sleep(delay)
            dxl_io.set_a1_traj2({id: coeffs[1]})
            time.sleep(delay)
            dxl_io.set_a2_traj2({id: coeffs[2]})
            time.sleep(delay)
            break
        except:
            errorCounter = errorCounter + 1
            print("nb errors = ", errorCounter)
            break


def setTorque1(id, duration, coeffs):
    errorCounter = 0
    delay = 0.001
    while True:
        try:
            dxl_io.set_torque1_size({id: 3})
            time.sleep(delay)
            dxl_io.set_duration1({id: duration})
            time.sleep(delay)
            dxl_io.set_a0_torque1({id: coeffs[0]})
            time.sleep(delay)
            dxl_io.set_a1_torque1({id: coeffs[1]})
            time.sleep(delay)
            dxl_io.set_a2_torque1({id: coeffs[2]})
            time.sleep(delay)
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
            dxl_io.set_torque2_size({id: 3})
            time.sleep(delay)
            dxl_io.set_duration2({id: duration})
            time.sleep(delay)
            dxl_io.set_a0_torque2({id: coeffs[0]})
            time.sleep(delay)
            dxl_io.set_a1_torque2({id: coeffs[1]})
            time.sleep(delay)
            dxl_io.set_a2_torque2({id: coeffs[2]})
            time.sleep(delay)
            break
        except:
            errorCounter = errorCounter + 1
            # print "Nope :/"
            pass
#         print "Nb errors : ", errorCounter

# ID_LIST = [1, 2, 3, 4]
# ID_SIZE = len(ID_LIST)

# DXL_DICT_1      = dict(zip(ID_LIST, [1]*ID_SIZE))
# DXL_DICT_0      = dict(zip(ID_LIST, [0]*ID_SIZE))
# DXL_DICT_PID    = dict(zip(ID_LIST, [[1,0,0]]*ID_SIZE))
    
# dxl_io.set_mode_dynaban(DXL_DICT_1)
# time.sleep(0.1)
# dxl_io.enable_torque(DXL_DICT_1)
# time.sleep(0.1)
# dxl_io.set_pid_gain(DXL_DICT_PID)
# time.sleep(0.1)

# print(dxl_io.get_moving_speed({num1:10}))


print ("Test with PID only:")
# dxl_io.set_mode_dynaban({num1:0})
# time.sleep(0.1)
# dxl_io.enable_torque({num1:1})
# time.sleep(0.1)



# dxl_io.set_pid_gain({num1:[1,0,0]})

dxl_io.set_mode_dynaban({num2:0})
time.sleep(0.1)
dxl_io.enable_torque({num2:1})
time.sleep(0.1)


dxl_io.set_goal_position({num2:0})
time.sleep(2)
# dxl_io.set_goal_position({num2:0})

# time_current = time.time()
# while (time.time()-time_current) <= 0.5:
#     print(dxl_io.get_present_speed([num1]))



# time.sleep(1)
time_current1 = time.time()
time_current2 = time.time()
setTraj1(num1, 20000, [2048.0,0.0, 0.0])
dxl_io.set_mode_dynaban({num1:3}) 

# while (time.time()-time_current1) <= 4:
# #             print(time.time() - time_current2)
#             if (time.time() - time_current2) > 0.04:
#                 ang = dxl_io.get_present_position([2]) + dxl_io.get_outputTorque([2])
#                 print(ang)
#                 time_current2 = time.time()

# setTraj1(num2, 5000, [2048.0, 0.0, 0.0])
# print ("Setting mode and tracking :")



# time_start = time.time()
setTraj2(num1, 20000, [2048.0, 512.0, 0.0])
# setTraj2(num2, 20000, [2048.0, 512.0, 0.0])
dxl_io.set_copy_next_buffer({num1:1})
time_current = time.time()
while (time.time()-time_current) <= 2:
#             print((time.time()-time_current))
            str_state = [str(dxl_io.get_present_position([num1])[0]),str(dxl_io.get_outputTorque([num1])[0])]
            state_file.write(",".join(str_state) + "\n")
            time.sleep(0.025)

setTraj2(num1, 20000, [3072.0, -512.0, 0.0])
# setTraj2(num2, 20000, [3072.0, -512.0, 0.0])
dxl_io.set_copy_next_buffer({num1:1})

time_current = time.time()
while (time.time()-time_current) <= 2:
#             print((time.time()-time_current))
            str_state = [str(dxl_io.get_present_position([num1])[0]),str(dxl_io.get_outputTorque([num1])[0])]
            state_file.write(",".join(str_state) + "\n")
            time.sleep(0.025)
        
        
setTraj2(num1, 10000, [2048.0, -512.0, 0.0])
# setTraj2(num2, 10000, [2048.0, -512.0, 0.0])
dxl_io.set_copy_next_buffer({num1:1})

time_current = time.time()
while (time.time()-time_current) <= 3:
#             print((time.time()-time_current))
            str_state = [str(dxl_io.get_present_position([num1])[0]),str(dxl_io.get_outputTorque([num1])[0])]
            state_file.write(",".join(str_state) + "\n")
            time.sleep(0.025)
time_end = time.time()
# print(time_end-time_start)
      






