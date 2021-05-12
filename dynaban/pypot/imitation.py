import time
import json

# pypot imports
import pypot.robot
import pypot.dynamixel
import matplotlib.pyplot as plt
import manipulator_math_utils_rbdl as mmu
import numpy as np
import sys
import csv

import pprint
pp = pprint.PrettyPrinter(indent=4)

SPLINE = 10000
MOTOR_ID = [1,2,4]
# GRIPPER_ID = 5
JOINTS = len(MOTOR_ID)
DXL_DICT_3      = dict(zip(MOTOR_ID, [3]*JOINTS))
DXL_DICT_1      = dict(zip(MOTOR_ID, [1]*JOINTS))
DXL_DICT_0      = dict(zip(MOTOR_ID, [0]*JOINTS))

csv_file = sys.argv[1]
urdf_file = "manipulator4_gripper.urdf"
transformations = [[-1,180],[1,180],[1,180],[-1,180]]
# transformations = [[-1,180], [-1,180]]
# transformations = [[-1,180],[-1,180],[-1,180]]
my_manipulator_math_utils = mmu.manipulator_math_utils(JOINTS)
# coeff_angle, coeff_torque = my_manipulator_math_utils.calculate_coeffs(csv_file,False,[[1,90]])
# coeff_angle, coeff_torque = my_manipulator_math_utils.calculate_coeffs(csv_file,False,[[-1,270],[-1,270],[1,180]])
# transformations = [[-1,180],[1,180]] P and Y 

# is_with_torque = False
# if is_with_torque:
#     coeff_angle, coeff_torque = my_manipulator_math_utils.calculate_coeffs(csv_file, False, transformations, with_torque=is_with_torque)
# else:
#     coeff_angle = my_manipulator_math_utils.calculate_coeffs(csv_file, False, transformations, with_torque=is_with_torque)

is_with_torque = False
if is_with_torque:
    coeff_angle, coeff_torque = my_manipulator_math_utils.calculate_coeffs(csv_file, transformations, urdf_file)
else:
    coeff_angle = my_manipulator_math_utils.calculate_coeffs(csv_file, transformations)

# pp.pprint(coeff_angle)

# gripper_state = np.genfromtxt(csv_file, delimiter=',')[:,]

start_angles = my_manipulator_math_utils.start_angles
# pp.pprint(coeff_angle)
# pp.pprint(coeff_torque)

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
ports = pypot.dynamixel.get_available_ports()
# state_file = open("demo2_imitation_wt.csv", "w")
state_file = open('demo2_imitation_wt_final.csv', 'w')
str_state = []

if not ports:
    raise IOError('no port found!')

print('ports found', ports)

print('connecting on the first available port:', ports[0])

dxl_io = pypot.dynamixel.DxlIO(ports[0],baudrate = 1000000)
# dxl_io.dxl_to_baudrate(1)
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
            
    # for joints in range(len(id)):
    # num = id[joints]
#         print(int(coeff_angle[id.index(motor)][0][4]*360/4096-180))
    dxl_io.set_mode_dynaban(DXL_DICT_0)
    time.sleep(0.1)
    dxl_io.enable_torque(DXL_DICT_1)
    dxl_io.enable_torque({3:1})
    time.sleep(0.1)
    
#         print(joints)
        # dxl_io.set_pid_gain({id[joints]:[1,0,0]})
        # time.sleep(0.1)

def go_to_start_pos(id, init_angle):
        
    for joints in range(len(id)):
        num = id[joints]
        start_angle = int(init_angle[joints])
        print("start angle",start_angle)
        current_pos = int(dxl_io.get_present_position([num])[0])
        diff = abs(current_pos-start_angle)
        print("diff", diff)
        for i in range(diff):
            if (current_pos > start_angle):
                dxl_io.set_goal_position({id[joints]:current_pos - i})
                time.sleep(0.05) 
            else:
                dxl_io.set_goal_position({id[joints]:current_pos + i})
                time.sleep(0.05)
# print(start_angles)
init_angle = [start_angles[0] + 90,start_angles[1], -start_angles[2]]
# init_angle = [-start_angles[0],-start_angles[1]]

init(MOTOR_ID)
# print(coeff_angle[0][0])

go_to_start_pos(MOTOR_ID, init_angle)

recorded_angles = []     
# data_all = []

def execute():
    last_flag = True

    print("in execute")
    for traj in range(len(coeff_angle[0])):
        if traj == 0:
            temp = time.time()
            for joints in range(len(MOTOR_ID)):
            # joints = 3    
                setTraj1(MOTOR_ID[joints],SPLINE, [coeff_angle[joints][traj][3],coeff_angle[joints][traj][2],coeff_angle[joints][traj][1],coeff_angle[joints][traj][0]])
                if is_with_torque:   
                    setTorque1(MOTOR_ID[joints],SPLINE, [coeff_torque[joints][traj][3],coeff_torque[joints][traj][2],coeff_torque[joints][traj][1],coeff_torque[joints][traj][0]])

            dxl_io.set_mode_dynaban(DXL_DICT_3) 
#             dxl_io.set_mode_dynaban({1:3, 2:3})


        else:
            for joints in range(len(MOTOR_ID)):
            # joints = 3
                setTraj2(MOTOR_ID[joints],SPLINE, [coeff_angle[joints][traj][3],coeff_angle[joints][traj][2],coeff_angle[joints][traj][1],coeff_angle[joints][traj][0]])
                if is_with_torque:
                    setTorque2(MOTOR_ID[joints],SPLINE, [coeff_torque[joints][traj][3],coeff_torque[joints][traj][2],coeff_torque[joints][traj][1],coeff_torque[joints][traj][0]])
            # print("elbow coeff: ",coeff_angle[3][traj])

            dxl_io.set_copy_next_buffer(DXL_DICT_1)
#             dxl_io.set_copy_next_buffer({1:1, 2:1})
            # dxl_io.set_mode_dynaban({3:3, 4:3})
            # dxl_io.set_mode_dynaban(DXL_DICT_3)
            # print("get a1 for traj1" ,dxl_io.get_a0_traj1([4]))
            # print("get a1 for traj2" ,dxl_io.get_a0_traj2([4]))
            start_time = time.time()
            while(time.time() - start_time < 1):
                
                # recorded_angles.append(dxl_io.get_present_position([3]))
#                 print(dxl_io.get_present_position([2])[0])
                time.sleep(0.04)
                pass

            # print(dxl_io.get_present_position([1]))
#             traj1 = []
#             for joints in range(len(MOTOR_ID)):
                
#                 traj1.append([dxl_io.get_a0_traj1([MOTOR_ID[joints]])[0],dxl_io.get_a1_traj1([MOTOR_ID[joints]])[0],dxl_io.get_a2_traj1([MOTOR_ID[joints]])[0],dxl_io.get_a3_traj1([MOTOR_ID[joints]])[0]])
            
#             traj2 = []
#             for joints in range(len(MOTOR_ID)):
#                     traj2.append([dxl_io.get_a0_traj2([MOTOR_ID[joints]])[0],dxl_io.get_a1_traj2([MOTOR_ID[joints]])[0],dxl_io.get_a2_traj2([MOTOR_ID[joints]])[0],dxl_io.get_a3_traj2([MOTOR_ID[joints]])[0]])
                    
#             if last_flag:
#                 prev_traj2 = traj1
#                 last_flag = False
# #                     print(prev_traj2[0][0])  
# #                     dxl_io.set_a0_traj1({1: 2306.0})
                
            
# #                     print("traj1" ,traj1)
# #                     print("prev_traj2" , prev_traj2)
    
#             for i in range(len(MOTOR_ID)):
#                 if prev_traj2[i] != traj1[i]:
                
#                 #    print("#############################")
# #                                 print("not same")
# #                                 reset_motor()
# #                                 clear_reg()
#                     rewrite_reg(i, prev_traj2)
#                     dxl_io.set_mode_dynaban({MOTOR_ID[i]:3})
            

                    
                    
# #                         dxl_io.set_mode_dynaban({1:3})
                
# #                         
#             prev_traj2 = []
#             for joints in range(len(MOTOR_ID)):
#                     prev_traj2.append([dxl_io.get_a0_traj2([MOTOR_ID[joints]])[0],dxl_io.get_a1_traj2([MOTOR_ID[joints]])[0],dxl_io.get_a2_traj2([MOTOR_ID[joints]])[0],dxl_io.get_a3_traj2([MOTOR_ID[joints]])[0]])

#             time.sleep(1)

#             time_current1 = time.time()
#             time_current2 = time.time()


#             while (time.time()-time_current1) <= 1:

# #                 if (time.time() - time_current2) > 0.043:
#                     # timestamp = (time.time()-temp)
#                     # data_all.append(timestamp)
#                     ang = dxl_io.get_present_position([2,3]) + dxl_io.get_outputTorque([2,3])
#                     print(ang)
#                     data_all.append(ang)
#         #             prin
                    
#                     time_current2 = time.time()
                # pp.pprint(data_all)
                # print(counter)
            
        
          

# for every_tuple in data_all:
#     state_file.write(",".join(every_tuple) + "\n")
# print(len(data_all))

    

# print(data_all)

execute()

# np.savetxt('demo3_angle.csv', recorded_angles, delimiter=',')

# with state_file:    
#     write = csv.writer(state_file)
#     write.writerows(data_all)             
#     time.sleep(1)