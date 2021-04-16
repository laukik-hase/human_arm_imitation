import math
import matplotlib.pyplot as plt
import numpy as np 
from scipy.optimize import curve_fit
import csv

def func2(t, b, c, d, e):
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
    angle1,angle2 = [],[]
    final_angle1,final_angle2 = [],[]
    # res2, t2 =[], []
    # res3, t3 =[], []
    for count in range(len(data)):
        angle1.append(data[count][1])
        angle2.append(data[count][2])

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
            t.append(final_angle1[element])
            t1.append(final_angle2[element])
            # t2.append(element[3])
            # t3.append(element[4])
        # cp.append(element[0])
    res.append(t)
    res1.append(t1)
    return res, res1
    # return res ,res1 ,res2 ,res3
# main Program
file_name = input('Enter csv file for motor: ')
angle1, angle2 = read_file(file_name)
# print(angle1)
# print(angle2)
# angle1, angle2, angle3, angle4 = read_file(file_name)
# print(angle2)
all_coeff = {}
coeff1 = {}
pcov1 = {}
count = 0
for value in angle1:
    coeff1[count], pcov1[count] = curve_fit(func2, np.linspace(0,0.5,len(value)),value)
    count = count + 1

coeff2 = {}
pcov2 = {}
count1 = 0
for value1 in angle2:
    coeff2[count1], pcov2[count1] = curve_fit(func2, np.linspace(0,0.5,len(value1)),value1)
    count1 = count1 + 1

# coeff3 = {}
# pcov3 = {}
# count2 = 0
# for value2 in angle3:
#     coeff3[count2], pcov3[count2] = curve_fit(func2, np.linspace(0,0.5,len(value2)),value2)
#     count2 = count2 + 1

# coeff4 = {}
# pcov4 = {}
# count3 = 0
# for value3 in angle4:
#     coeff4[count3], pcov4[count3] = curve_fit(func2, np.linspace(0,0.5,len(value3)),value3)
#     count3 = count3 + 1

all_coeff[0] = coeff1
all_coeff[1] = coeff2
# all_coeff[2] = coeff3
# all_coeff[3] = coeff4

print(all_coeff)