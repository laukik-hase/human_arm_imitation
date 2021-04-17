import time
import numpy
import json

# pypot imports
import pypot.robot
import pypot.dynamixel

import math
import matplotlib.pyplot as plt
import numpy as np 
from scipy.optimize import curve_fit
import csv
import pypot.dynamixel
import sys

# motor_id = [2,4]
# size = len(motor_id)
# DXL_DICT_3      = dict(zip(motor_id, [3]*size))
# DXL_DICT_1      = dict(zip(motor_id, [3]*size))


# ports = pypot.dynamixel.get_available_ports()
# state_file = open("ps_pe_with_torque_1sec_ma_new-urdf.csv", "w")
# str_state = []

# if not ports:
#     raise IOError('no port found!')

# print('ports found', ports)

# print('connecting on the first available port:', ports[0])
# dxl_io = pypot.dynamixel.DxlIO(ports[0])

# def setTraj1(id, duration, coeffs):
#     errorCounter = 0
#     delay = 0.001
#     while True:
#         try:
#             dxl_io.set_traj1_size({id: 4})
#             time.sleep(delay)
#             dxl_io.set_duration1({id: duration})
#             time.sleep(delay)
#             dxl_io.set_a0_traj1({id: coeffs[0]})
#             time.sleep(delay)
#             dxl_io.set_a1_traj1({id: coeffs[1]})
#             time.sleep(delay)
#             dxl_io.set_a2_traj1({id: coeffs[2]})
#             time.sleep(delay)
#             dxl_io.set_a3_traj1({id: coeffs[3]})
#             time.sleep(delay)
# #             dxl_io.set_a4_traj1({id: coeffs[4]})
# #             time.sleep(delay)

#             break
#         except:
#             errorCounter = errorCounter + 1
#             # print "Nope :/"
#             break
#             print("nb errors1 = ", errorCounter)


# def setTraj2(id, duration, coeffs):
#     errorCounter = 0
#     delay = 0.001

#     while True:
#         try:
#             dxl_io.set_traj2_size({id: 4})
#             time.sleep(delay)
#             dxl_io.set_duration2({id: duration})
#             time.sleep(delay)
#             dxl_io.set_a0_traj2({id: coeffs[0]})
#             time.sleep(delay)
#             dxl_io.set_a1_traj2({id: coeffs[1]})
#             time.sleep(delay)
#             dxl_io.set_a2_traj2({id: coeffs[2]})
#             time.sleep(delay)
#             dxl_io.set_a3_traj2({id: coeffs[3]})
#             time.sleep(delay)
# #             dxl_io.set_a4_traj2({id: coeffs[4]})
# #             time.sleep(delay)

#             break
#         except:
#             errorCounter = errorCounter + 1
#             print("nb errors2 = ", errorCounter)
#             break


# def setTorque1(id, duration, coeffs):
#     errorCounter = 0
#     delay = 0.001
#     while True:
#         try:
#             dxl_io.set_torque1_size({id: 4})
#             time.sleep(delay)
#             dxl_io.set_duration1({id: duration})
#             time.sleep(delay)
#             dxl_io.set_a0_torque1({id: coeffs[0]})
#             time.sleep(delay)
#             dxl_io.set_a1_torque1({id: coeffs[1]})
#             time.sleep(delay)
#             dxl_io.set_a2_torque1({id: coeffs[2]})
#             time.sleep(delay)
#             dxl_io.set_a3_torque1({id: coeffs[3]})
#             time.sleep(delay)
# #             dxl_io.set_a4_torque1({id: coeffs[4]})
# #             time.sleep(delay)
#             break
#         except:
#             errorCounter = errorCounter + 1
#             # print "Nope :/"
#             pass
# #         print "Nb errors : ", errorCounter

# def setTorque2(id, duration, coeffs):
#     errorCounter = 0
#     delay = 0.001
#     while True:
#         try:
#             dxl_io.set_torque2_size({id: 4})
#             time.sleep(delay)
#             dxl_io.set_duration2({id: duration})
#             time.sleep(delay)
#             dxl_io.set_a0_torque2({id: coeffs[0]})
#             time.sleep(delay)
#             dxl_io.set_a1_torque2({id: coeffs[1]})
#             time.sleep(delay)
#             dxl_io.set_a2_torque2({id: coeffs[2]})
#             time.sleep(delay)
#             dxl_io.set_a3_torque2({id: coeffs[3]})
#             time.sleep(delay)
# #             dxl_io.set_a4_torque2({id: coeffs[4]})
# #             time.sleep(delay)
#             break
#         except:
#             errorCounter = errorCounter + 1
#             # print "Nope :/"
#             pass

def eval_poly(t, b, c, d, e):
    return b*pow(t, 3) + c*pow(t, 2) + d*t + e

def read_file(inp):
    
    data = []
    cp = []
    with open(inp, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            data.append(list(map(float, row)))
    
    res, t = [], []
    res1, t1 =[], []
    angle1,angle2,torque1,torque2 = [],[],[],[]
    final_angle1,final_angle2 = [],[]
    res2, t2 =[], []
    res3, t3 =[], []
    for count in range(len(data)):
        angle1.append(data[count][1])
        angle2.append(data[count][2])
        torque1.append(data[count][3])
        torque2.append(data[count][4])

    window_size = 5

    i = 0
    counter = 0
    moving_averages = []
    moving_averages1 = []
    while i < len(angle1) - window_size + 1:
        this_window = angle1[i : i + window_size]
        this_window1 = angle2[i : i + window_size]
        window_average = sum(this_window) / window_size
        window_average1 = sum(this_window1) / window_size
        moving_averages.append(window_average)
        moving_averages1.append(window_average1)
        i += 1

    while counter < window_size:
        final_angle1.append(moving_averages[0])
        final_angle2.append(moving_averages1[0])
        counter = counter + 1

    final_angle1 = final_angle1 + moving_averages[1:]
    final_angle2 = final_angle2 + moving_averages1[1:]
    
    # print(final_angle1)
    k = 1
    for element in range(len(data)):
        if data[element][0] <= k * 1:
            t.append(final_angle1[element])
            t1.append(final_angle2[element])
            t2.append(torque1[element])
            t3.append(torque2[element])
        else:
            k = k + 1
            res.append(t)
            res1.append(t1)
            res2.append(t2)
            res3.append(t3)
            t = []
            t1 = []
            t2 = []
            t3 = []
            t.append(final_angle1[element])
            t1.append(final_angle2[element])
            t2.append(torque1[element])
            t3.append(torque2[element])
        # cp.append(element[0])
    res.append(t)
    res1.append(t1)
    res2.append(t2)
    res3.append(t3)
#     return res, res1
    return res ,res1 ,res2 ,res3
# file_name = input('Enter csv file for motor: ')
angle1, angle2, torque1, torque2 = read_file('Ps_Pe_slow.csv')

# angle1, angle2 = read_file('Ps_Pe_new (1).csv')
# print(angle2)
all_coeff = {}
coeff1 = {}
pcov1 = {}
count = 0
for value in angle1:
    
    coeff1[count], pcov1[count] = curve_fit(eval_poly, np.linspace(0,1,len(value)),value)
    count = count + 1

coeff2 = {}
pcov2 = {}
count1 = 0
for value1 in angle2:
    coeff2[count1], pcov2[count1] = curve_fit(eval_poly, np.linspace(0,1,len(value1)),value1)
    count1 = count1 + 1

coeff3 = {}
pcov3 = {}
count2 = 0
for value2 in torque1:
    coeff3[count2], pcov3[count2] = curve_fit(eval_poly, np.linspace(0,1,len(value2)),value2)
    count2 = count2 + 1

coeff4 = {}
pcov4 = {}
count3 = 0
for value3 in torque2:
    coeff4[count3], pcov4[count3] = curve_fit(eval_poly, np.linspace(0,1,len(value3)),value3)
    count3 = count3 + 1

all_coeff[0] = coeff1
all_coeff[1] = coeff2
all_coeff[2] = coeff3
all_coeff[3] = coeff4


print(all_coeff) 

# print(all_coeff[2][0][4])
# print(all_coeff[3][0][4])

# dxl_io.set_mode_dynaban({3:0})
# time.sleep(0.1)
# dxl_io.enable_torque({3:1})
# time.sleep(0.1)

# dxl_io.set_goal_position({3:90})
# time.sleep(1)
# dxl_io.set_pid_gain({3:[1,0,0]})
# print(all_coeff)

# start_angle = int(all_coeff[0][0][4]*360/4096-270)
# print(start_angle)

# def init(id):
#     for joints in range(len(id)):
#         num = id[joints]
# #         print(int(all_coeff[id.index(motor)][0][4]*360/4096-180))
#         dxl_io.set_mode_dynaban({num:0})
#         time.sleep(0.1)
#         dxl_io.enable_torque({num:1})
#         time.sleep(0.1)
# #         print(joints)
#         start_angle = int(all_coeff[joints][0][3]*360/4096-180)
# #         print(start_angle)
#         current_pos = int(dxl_io.get_present_position([num])[0])
# #         print(current_pos)
#         diff = abs(current_pos-start_angle)
#         for i in range(diff):
#              if (current_pos > start_angle):
#                       dxl_io.set_goal_position({id[joints]:current_pos - i})
#                       time.sleep(0.01) 
#              else:
#                       dxl_io.set_goal_position({id[joints]:current_pos + i})
#                       time.sleep(0.01)
                  
# # #         time.sleep(0.5)
#         dxl_io.set_pid_gain({id[joints]:[1,0,0]})
#         time.sleep(0.1)

# init(motor_id)

# # # for i in range(10):
# #     for j in range(2):
# #         timer = [str(i),str(i+1)]
# #         str_state.extend(timer) 
        
# #     state_file.write(",".join(str_state) + "\n")
# #     print(timer)
# condition = True
# for traj in range(0,len(coeff1)):
#     if traj == 0:
#         for joints in range(len(motor_id)):
#             joint_torque = joints + len(motor_id)
# #             print("hi")
# #             print(all_coeff[joint_torque][traj][4])
#             setTraj1(motor_id[joints],10000, [all_coeff[joints][traj][3],all_coeff[joints][traj][2],all_coeff[joints][traj][1],all_coeff[joints][traj][0]])
            
#             setTorque1(motor_id[joints],10000, [all_coeff[joint_torque][traj][3],all_coeff[joint_torque][traj][2],all_coeff[joint_torque][traj][1],all_coeff[joint_torque][traj][0]])
            
#         dxl_io.set_mode_dynaban({4:3,2:3}) 
        
#     else:
#         for joints in range(len(motor_id)):
            
#             joint_torque = joints + len(motor_id)
# #             print(all_coeff[joint_torque][traj][0])
#             setTraj2(motor_id[joints],10000, [all_coeff[joints][traj][3],all_coeff[joints][traj][2],all_coeff[joints][traj][1],all_coeff[joints][traj][0]])
            
#             setTorque2(motor_id[joints],10000, [all_coeff[joint_torque][traj][3],all_coeff[joint_torque][traj][2],all_coeff[joint_torque][traj][1],all_coeff[joint_torque][traj][0]])
            
#         dxl_io.set_copy_next_buffer({4:1,2:1})
# #         time.sleep(1)
#         time_current = time.time()
        
#         while (time.time()-time_current) <= 1:
#             if condition == True:
#                 temp = time.time()
#             else:
#                 pass
#             condition = False
#             time_stamp = [str(time.time()-temp)]
# #             print(time_stamp)
#             str_state.extend(time_stamp)
            
#             for joints in motor_id:
#                 ang = [str(dxl_io.get_present_position([joints])[0]),str(dxl_io.get_outputTorque([joints])[0])]
#                 print(ang)

#                 str_state.extend(ang)
            
#             state_file.write(",".join(str_state) + "\n")
#             str_state = []
#             time.sleep(0.025)


# # time_elapsed = (time.clock() - time_start)



