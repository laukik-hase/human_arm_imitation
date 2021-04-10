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

num1 = 2
num2 = 4

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

def func2(t, c, d, e, f):
    return  c*pow(t, 3) + d*pow(t, 2) + e*t + f

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

# file_name = input('Enter csv file for motor: ')
angle1, angle2 = read_file('Ps_Pe_new.csv')
# angle1, angle2, angle3, angle4 = read_file(file_name)

coeff1 = {}
pcov1 = {}
count1 = 0

for value in angle1:
    coeff1[count1], pcov1[count1] = curve_fit(func2, np.linspace(0,0.5,len(value)),value)
#     print(coeff1[count1],count1)
    count1 = count1 + 1

coeff2 = {}
pcov2 = {}
count2 = 0

for value in angle2:
    coeff2[count2], pcov2[count2] = curve_fit(func2, np.linspace(0,0.5,len(value)),value)
#     print(coeff2[count2],count2)
    count2 = count2 + 1


print ("Test with PID only:")
dxl_io.set_mode_dynaban({num1:0})
time.sleep(0.1)
dxl_io.enable_torque({num1:1})
time.sleep(0.1)
# dxl_io.enable_torque({1:1})
time.sleep(0.1)
dxl_io.set_mode_dynaban({num2:0})
time.sleep(0.1)
dxl_io.enable_torque({num2:1})
time.sleep(0.1)


dxl_io.set_goal_position({num1:-75})
dxl_io.set_goal_position({num2:0})
time.sleep(2)
dxl_io.set_pid_gain({num1:[1,0,0]})
time.sleep(0.1)
dxl_io.set_pid_gain({num2:[2,0,0]})
time.sleep(0.1)
# print ("Setting traj1 :")

# print(dxl_io.get_goal_position([num]))

# setTraj1(num1,5000, [2048.0,0.0,0.0,0.0,0.0,0.0,0.0])
# dxl_io.set_mode_dynaban({num1:3}) 
# time.sleep(1)
# print(dxl_io.get_goal_position([num]))

for i in range(0,len(coeff1)):
    if i == 0:
        setTraj1(num1,5000, [coeff1[i][3],coeff1[i][2],coeff1[i][1],coeff1[i][0]])
        setTorque1(num1,5000, [0.512,0.0,0.0])
        setTraj1(num2,5000, [coeff2[i][3],coeff2[i][2],coeff2[i][1],coeff2[i][0]])
        setTorque1(num2,5000, [0.512,0.0,0.0])           
        dxl_io.set_mode_dynaban({num2:3,num1:3}) 
#         time.sleep(1)
    else:
        setTraj2(num1,5000, [coeff1[i][3],coeff1[i][2],coeff1[i][1],coeff1[i][0]])
        if i > 10
        setTorque2(num1,5000, [0.512,0.512,0.0])
        setTraj2(num2,5000, [coeff2[i][3],coeff2[i][2],coeff2[i][1],coeff2[i][0]])
        setTorque2(num2,5000, [0.512,0.512,0.0])
        dxl_io.set_copy_next_buffer({num2:1,num1:1})
        time.sleep(0.5)






