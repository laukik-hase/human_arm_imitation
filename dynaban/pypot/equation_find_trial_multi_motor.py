import math
import matplotlib.pyplot as plt
import numpy as np 
from scipy.optimize import curve_fit
import csv

def func2(t, a, b, c, d):
    return a*pow(t, 3) + b*pow(t, 2) + c*t + d

def read_file(inp):
    
    data = []
    cp = []
    with open(inp, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            data.append(list(map(int, row)))
    res, t = [], []
    k = 1

    for element in data:
        if element[0] <= k * 500:
            t.append(element[1])
        else:
            k = k + 1
            res.append(t)
            t = []
            t.append(element[1])
        cp.append(element[0])
    return res ,cp

# main Program
file_name1 = input('Enter csv file for motor 1: ')
angle1, timestamp1 = read_file(file_name1)
file_name2 = input('Enter csv file for motor 2: ')
angle2, timestamp2 = read_file(file_name2)
# file_name3 = input('Enter csv file for motor 3: ')
# angle3, timestamp3 = read_file(file_name3)
# file_name4 = input('Enter csv file for motor 4: ')
# angle4, timestamp4 = read_file(file_name4) 

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