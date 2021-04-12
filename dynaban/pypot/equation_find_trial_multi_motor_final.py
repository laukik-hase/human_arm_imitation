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