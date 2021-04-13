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

motor_id = [2,3]

ports = pypot.dynamixel.get_available_ports()
state_file = open("path.csv", "w")


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
            print("nb errors1 = ", errorCounter)


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
            print("nb errors2 = ", errorCounter)
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
def eval_poly(t, a0, a1, a2, a3, a4):
    return a0*pow(t, 4) + a1*pow(t, 3) + a2*pow(t, 2) + a3*t + a4

def read_file(inp):
    
    data = []
    cp = []
    with open(inp, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            data.append(list(map(int, row)))
    res, t = [], []
    res1, t1 =[], []
    # res2, t2 =[], []
    # res3, t3 =[], []
    k = 1

    for element in data:
        if element[0] <= k * 500:
            t.append(element[1])
            t1.append(element[2])
            # t2.append(element[3])
            # t3.append(element[4])
        else:
            k = k + 1
            res.append(t)
            res1.append(t1)
            # res2.append(t1)
            # res3.append(t1)
            t = []
            t1 = []
            # t2 = []
            # t3 = []
            t.append(element[1])
            t1.append(element[2])
            # t2.append(element[3])
            # t3.append(element[4])
        cp.append(element[0])
    return res ,res1
    # return res ,res1 ,res2 ,res3
# main Program
file_name = input('Enter csv file for motor: ')
angle1, angle2 = read_file(file_name)
# angle1, angle2, angle3, angle4 = read_file(file_name)

all_coeff = {}
coeff1 = {}
pcov1 = {}
coeff2 = {}
pcov2 = {}
coeff3 = {}
pcov3 = {}
coeff4 = {}
pcov4 = {}
count = 0

for value in angle1:
    coeff1[count], pcov1[count] = curve_fit(func2, np.linspace(0,0.5,len(value)),value)
    coeff2[count], pcov2[count] = curve_fit(func2, np.linspace(0,0.5,len(value)),value)
    #coeff3[count], pcov3[count] = curve_fit(func2, np.linspace(0,0.5,len(value)),value)
    #coeff4[count], pcov4[count] = curve_fit(func2, np.linspace(0,0.5,len(value)),value)
    count = count + 1
all_coeff[0] = coeff1
all_coeff[1] = coeff2
#all_coeff[2] = coeff3
#all_coeff[3] = coeff4
print(all_coeff[0][0])

# TODO : Put all delays in one variable
def init (id)
    for motor in id:
        dxl_io.set_mode_dynaban({motor:0})
        time.sleep(0.1)
        dxl_io.enable_torque({motor:1})
        time.sleep(0.1)
        dxl_io.set_goal_position({motor:(all_coeff[id.index(i)][0][3]*360/4096-180)})
        time.sleep(0.5)
        dxl_io.set_pid_gain({motor:[1,0,0]})
        time.sleep(0.1)

init(motor_id)

for traj in range(0,len(coeff1)):
    if traj == 0:
        for joints in range(len(motor_id)):
            setTraj1(motor_id[joints],5000, [all_coeff[joints][traj][4],all_coeff[j][traj][3],all_coeff[joints][traj][2],all_coeff[joints][traj][1],all_coeff[joints][traj][0]])
        dxl_io.set_mode_dynaban({2:3,3:3}) 
        
    else:
        for joints in range(len(motor_id)):
            setTraj2(motor_id[joints],5000, [all_coeff[joints][traj][4],all_coeff[j][traj][3],all_coeff[joints][traj][2],all_coeff[joints][traj][1],all_coeff[joints][traj][0]])
        
        dxl_io.set_copy_next_buffer({2:1,3:1})
        
        time_current = time.time()
        while (time.time()-time_current) <= 0.5:
            for joints in range(len(motor_id)):
                str_state.extend[(str(dxl_io.get_present_position([joints])[0]),str(dxl_io.get_outputTorque([joints])[0]))] 

            state_file.write(",".join(str_state) + "\n")

time_elapsed = (time.clock() - time_start)



