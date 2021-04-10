import math
import matplotlib.pyplot as plt
import numpy as np 
from scipy.optimize import curve_fit
import csv

def func2(t, a, b, c, d, e, f):
    return a*pow(t, 5) + b*pow(t, 4) + c*pow(t, 3) + d*pow(t, 2) + e*t + f

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

coeff1 = {}
pcov1 = {}
count1 = 0

for value in angle1:
    coeff1[count1], pcov1[count1] = curve_fit(func2, np.linspace(0,0.5,len(value)),value)
    print(coeff1[count1],count1)
    count1 = count1 + 1

coeff2 = {}
pcov2 = {}
count2 = 0

for value in angle2:
    coeff2[count2], pcov2[count2] = curve_fit(func2, np.linspace(0,0.5,len(value)),value)
    print(coeff2[count2],count2)
    count2 = count2 + 1

# coeff3 = {}
# pcov3 = {}
# count3 = 0

# for value in angle3:
#     coeff3[count3], pcov3[count3] = curve_fit(func2, np.linspace(0,0.5,len(value)),value)
#     print(coeff3[count3],count3)
#     count3 = count3 + 1

# coeff4 = {}
# pcov4 = {}
# count4 = 0

# for value in angle4:
#     coeff4[count4], pcov4[count4] = curve_fit(func2, np.linspace(0,0.5,len(value)),value)
#     print(coeff4[count4],count4)
#     count4 = count4 + 1