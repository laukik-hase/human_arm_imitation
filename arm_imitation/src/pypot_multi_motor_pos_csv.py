# Code to receive angle values on ros topic /joint_angles
# and move the manipulator based on those angles
# using pypot library

import time
import sys
# pypot imports
import pypot.dynamixel

def setTraj1(id, duration, coeffs):
    errorCounter = 0
    delay = 0.001
    while True:
        try:
            dxl_io.set_traj1_size({id: 5})
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
            dxl_io.set_a4_traj1({id: coeffs[4]})
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
            dxl_io.set_traj2_size({id: 5})
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
            dxl_io.set_a4_traj2({id: coeffs[4]})
            time.sleep(delay)
            break
        except:
            errorCounter = errorCounter + 1
            print("nb errors = ", errorCounter)
            break

offset = [180, 180, 180, 180]
ID_LIST = [1, 2, 3, 4]
ID_SIZE = len(ID_LIST)

DXL_DICT_1      = dict(zip(ID_LIST, [1]*ID_SIZE))
DXL_DICT_0      = dict(zip(ID_LIST, [0]*ID_SIZE))
DXL_DICT_PID    = dict(zip(ID_LIST, [[1,0,0]]*ID_SIZE))

ports = pypot.dynamixel.get_available_ports()

if not ports:
    raise IOError('no port found!')

print('ports found', ports)

print('connecting on the first available port:', ports[0])
dxl_io = pypot.dynamixel.DxlIO(ports[0])

dxl_io.set_mode_dynaban(DXL_DICT_1)
time.sleep(0.1)
dxl_io.enable_torque(DXL_DICT_1)
time.sleep(0.1)
dxl_io.set_pid_gain(DXL_DICT_PID)
time.sleep(0.1)

if ( len(sys.argv) == 2 ):
    file_name = sys.argv[1]
else:
    print("Usage: python csv_angle_publisher.py <csv_file_name>")
    exit()

at_initial_pos = False

with open(file_name, mode='r') as openfileobject:
    for line in openfileobject:
        str_state = line.split(',')
        state = [int( ((float(str_state[i+1]) + offset[i])%360)/360.0*4096 ) for i in range(ID_SIZE)]
        print(state)
        if not at_initial_pos:
            setTraj1(ID_LIST[3], 20000, [state[3], 0.0, 0.0, 0.0, 0.0])
            dxl_io.set_mode_dynaban({ID_LIST[3]:2})
            print("[+] Moving to initial pos")
            time.sleep(2)
            at_initial_pos = True
            print("[+] At initial pos")
        else:
            setTraj1(ID_LIST[3], 5000, [state[3], 0.0, 0.0, 0.0, 0.0])
            # dxl_io.set_copy_next_buffer({ID_LIST[3]:1})
            dxl_io.set_mode_dynaban({ID_LIST[3]:2})
            time.sleep(0.5)

dxl_io.set_mode_dynaban(DXL_DICT_0)
time.sleep(0.1)
dxl_io.enable_torque(DXL_DICT_0)
time.sleep(0.1)