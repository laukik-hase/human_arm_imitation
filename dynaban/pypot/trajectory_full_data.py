import math
import matplotlib.pyplot as plt
import numpy as np 
from scipy.optimize import curve_fit
import csv

def func2(t, a, b, c, d):
    return a*pow(t, 3) + b*pow(t, 2) + c*t + d

def read_file():
    
    data = []
    cp = []
    with open('data.csv', 'r') as file:
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
angle, timestamp = read_file()
#print(timestamp)
coeff = {}
pcov2 = {}
count = 0
# print(len(angle))
# print(range(1,len(angle[0])))
# print(angle[0])
# popt2, pcov2 = curve_fit(func2, np.linspace(1,,), angle[0])
# fit_cp1 = func1(np.array (temperature), *popt1)
#fit_cp2 = func2(np.array (angle), *popt2)
#print(popt2)
print(np.linspace(0,0.5,10))
for value in angle:
    # coeff[count], pcov2[count] = curve_fit(func2, value, np.linspace(0,0.5,len(value)))
    coeff[count], pcov2[count] = curve_fit(func2, np.linspace(0,0.5,len(value)),value)
    fit_cp2 = func2(np.array(value), *coeff[count])
    #print(value)

    print(coeff[count],count)
    #print(range(0,len(value)))
    count = count + 1
print(coeff[0][0])
# plt.figure()
# plt.plot(temperature, cp, color = 'blue', lw = 3)
# plt.plot (temperature, fit_cp2, color = 'green', lw = 3)
# plt. legend ( ['Actual data', 'cubical cure fit'])
# plt.title(' Cubical curve fit')
# plt.xlabel('Temperature[k]')
# plt.ylabel('cp')
# plt. show()