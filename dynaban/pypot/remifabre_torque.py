import time
import numpy
import json
import math
# pypot imports
import pypot.dynamixel

num = 4

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

def apna_delay(t):
    for i in range(t):
         time.sleep(0.1)
         pass
   
ID_LIST = [1, 2, 3, 4]
ID_SIZE = len(ID_LIST)

DXL_DICT_1      = dict(zip(ID_LIST, [1]*ID_SIZE))
DXL_DICT_0      = dict(zip(ID_LIST, [0]*ID_SIZE))
DXL_DICT_PID    = dict(zip(ID_LIST, [[1,0,0]]*ID_SIZE))
    
dxl_io.set_mode_dynaban(DXL_DICT_1)
time.sleep(0.1)
dxl_io.enable_torque(DXL_DICT_1)
time.sleep(0.1)
dxl_io.set_pid_gain(DXL_DICT_PID)
time.sleep(0.1)

print ("Test with PID only:")
dxl_io.set_mode_dynaban({num:0})
time.sleep(0.1)
dxl_io.enable_torque({num:1})
time.sleep(0.1)

dxl_io.set_goal_position({4:90})
time.sleep(1)
# dxl_io.set_pid_gain({4:[1,0,0]})
# time.sleep(0.1)

setTraj1(num, 10000, [3072.0, 0.0-512.0, 0.0])
# setTorque1(num, 10000, [3072.0, 0.0-512.0, 0.0])

print ("Setting mode and tracking :")

dxl_io.set_mode_dynaban({num:2}) 

print ("Sleeping")
# apna_delay(10)

for i in range(1):   
            print ("Traj2")
            setTraj2(num, 10000, [2560.0, -512.0, 0.0])
#             setTorque2(num, 10000, [2560.0, -512.0, 0.0])
#             
            dxl_io.set_copy_next_buffer({num:1})
            apna_delay(10)
#             time.sleep(1)
            
            print ("Traj3")
            setTraj2(num, 10000, [2048.0, -512.0, 0.0])
#             setTorque2(num, 10000, [2048.0, -512.0, 0.0])
            
            dxl_io.set_copy_next_buffer({num:1})
            # buffer 2 will be copied into buffer 1 once the traj 1 finishes
#             time.sleep(1)
            apna_delay(10)
    
            print ("Traj4")
            setTraj2(num, 10000, [1536.0,-512.0, 0.0])
#             setTorque2(num, 10000, [1536.0, -512.0, 0.0])
            
            dxl_io.set_copy_next_buffer({num:1})
             # buffer 2 will be copied into buffer 1 once the traj 1 finishes
#             time.sleep(1)
            apna_delay(10)
            
            print ("Traj5")
            setTraj2(num, 10000, [1024.0, 512.0, 0.0])
#             setTorque2(num, 10000, [1024.0, 512.0, 0.0])

            dxl_io.set_copy_next_buffer({num:1})
             # buffer 2 will be copied into buffer 1 once the traj 1 finishes
#             time.sleep(1)
            apna_delay(10)
    
            print ("Traj6")
            setTraj2(num, 10000, [1536.0, 512.0, 0.0])
#             setTorque2(num, 10000, [1536.0, 512.0, 0.0])

            dxl_io.set_copy_next_buffer({num:1}) # buffer 2 will be copied into buffer 1 once the traj 1 finishes
#             time.sleep(1)
            apna_delay(10)
    
            print ("Traj7")
            setTraj2(num, 10000, [2048.0, 512.0, 0.0])
#             setTorque2(num, 10000, [2048.0, 512.0, 0.0])

            dxl_io.set_copy_next_buffer({num:1})
 # buffer 2 will be copied into buffer 1 once the traj 1 finishes
#             time.sleep(1)
            apna_delay(10)
    
            print ("Traj8")
            setTraj2(num, 10000, [2560.0, 512.0, 0.0])
#             setTorque2(num, 10000, [2560.0, 512.0, 0.0])

            dxl_io.set_copy_next_buffer({num:1})
 # buffer 2 will be copied into buffer 1 once the traj 1 finishes
#             time.sleep(1)
            apna_delay(10)
    
            print ("Traj1")
            setTraj2(num, 10000, [3072.0, -512.0, 0.0])
#             setTorque2(num, 10000, [2560.0, -512.0, 0.0])

            dxl_io.set_copy_next_buffer({num:1})
# buffer 2 will be copied into buffer 1 once the traj 1 finishes
#             time.sleep(1)
            apna_delay(10)