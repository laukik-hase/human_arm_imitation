import time
import json

# pypot imports
import pypot.robot
import pypot.dynamixel
import matplotlib.pyplot as plt
import manipulator_math_utils as mmu
import sys


motor_id = [2,3,4]
JOINTS = len(motor_id)
DXL_DICT_3      = dict(zip(motor_id, [3]*JOINTS))
DXL_DICT_1      = dict(zip(motor_id, [3]*JOINTS))

filename = sys.argv[1]
my_manipulator_math_utils = mmu.manipulator_math_utils(JOINTS)
coeff_angle, coeff_torque = my_manipulator_math_utils.calculate_coeffs(filename,False, [[-1,90],[-1,270],[-1,180]])

# print(angle_coeff)

ports = pypot.dynamixel.get_available_ports()
state_file = open("demo2_imitation_wot.csv", "w")
str_state = []

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
def init(id):
    for joints in range(len(id)):
        num = id[joints]
#         print(int(coeff_angle[id.index(motor)][0][4]*360/4096-180))
        dxl_io.set_mode_dynaban({num:0})
        time.sleep(0.1)
        dxl_io.enable_torque({num:1})
        time.sleep(0.1)
#         print(joints)
        start_angle = int(coeff_angle[joints][0][3]*360/4096-180)
        print(start_angle)
        current_pos = int(dxl_io.get_present_position([num])[0])
#         print(current_pos)
        diff = abs(current_pos-start_angle)
        for i in range(diff):
             if (current_pos > start_angle):
                      dxl_io.set_goal_position({id[joints]:current_pos - i})
                      time.sleep(0.01) 
             else:
                      dxl_io.set_goal_position({id[joints]:current_pos + i})
                      time.sleep(0.01)
                  
#         time.sleep(0.5)
        dxl_io.set_pid_gain({id[joints]:[1,0,0]})
        time.sleep(0.1)

init(motor_id)

def print_trajectory(id):
    for joints in range(len(id)):
        print('\n')
        for j in range(len(coeff_angle[0])):
            print(int(coeff_angle[joints][j][3]*360/4096-180),end = '  ')
            
# print_trajectory(motor_id)                  
condition = True


for traj in range(len(coeff_angle[0])):
    if traj == 0:
        for joints in range(len(motor_id)):

            setTraj1(motor_id[joints],10000, [coeff_angle[joints][traj][3],coeff_angle[joints][traj][2],coeff_angle[joints][traj][1],coeff_angle[joints][traj][0]])
            
#             setTorque1(motor_id[joints],10000, [coeff_torque[joints][traj][3],coeff_torque[joints][traj][2],coeff_torque[joints][traj][1],coeff_torque[joints][traj][0]])
            
        dxl_io.set_mode_dynaban(DXL_DICT_3 ) 
        
    else:
        for joints in range(len(motor_id)):
            
#             print(coeff_angle[joints][traj][0])
            setTraj2(motor_id[joints],10000, [coeff_angle[joints][traj][3],coeff_angle[joints][traj][2],coeff_angle[joints][traj][1],coeff_angle[joints][traj][0]])
            
#             setTorque2(motor_id[joints],10000, [coeff_torque[joints][traj][3],coeff_torque[joints][traj][2],coeff_torque[joints][traj][1],coeff_torque[joints][traj][0]])
            
        dxl_io.set_copy_next_buffer(DXL_DICT_1 )
        
#         time.sleep(1)
        
        time_current1 = time.time()
        time_current2 = time.time()
        data_all = [] 
        while (time.time()-time_current1) <= 1:
#             print(dxl_io.get_present_position([2,3]))
#             if condition == True:
#                 temp = time.time()
#             else:
#                 pass
#             condition = False
#             time_stamp = [str(time.time()-temp)]
# #             print(time_stamp)
#             str_state.extend(time_stamp)
    #             print(dxl_io.get_outputTorque([2])[0])
            if time.time()-time_current2 >= 0.04: 
                for _ID in [2,3]:
                
                    ang = [str(dxl_io.get_present_position([_ID])[0])]
    #                 print(ang)

                    str_state.extend(ang)
#             data_all.append(str_state)
            
                state_file.write(",".join(str_state) + "\n")
                str_state = []
                time_current2 = time.time()
            

# print(data_all)
