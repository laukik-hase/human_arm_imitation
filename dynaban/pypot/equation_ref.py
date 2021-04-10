import math
import matplotlib.pyplot as plt
import numpy as np 
from scipy.optimize import curve_fit

# Curve fit function
def func1(t, a, b):
    return a*t + b

def func2(t, a, b, c, d):
    #print(a,b,c,d)
    return a*pow(t, 3) + b*pow(t, 2) + c*t + d

#Reading thermodynamic data file
def read_file():
    temperature = []
    cp = []
    for line in open ('/home/ameya/interpolation/trial.csv', 'r'):
        values = line.split(',')
        temperature.append(float(values[0]))
        cp.append(float(values[1]))
    return [temperature, cp]

# main Program
temperature, cp = read_file()
print(cp)
popt1, pcov1 = curve_fit(func1, temperature, cp)
popt2, pcov2 = curve_fit(func2, temperature, cp)
fit_cp1 = func1(np.array (temperature), *popt1)
fit_cp2 = func2(np.array (temperature), *popt2)
print(popt2)


# plotting the actual and estimated curves
plt. figure
plt.plot(temperature, cp, color = 'blue', lw = 3)
plt.plot(temperature, fit_cp1, color = 'red', lw = 3) 
plt.legend( ['Actual data', 'linear cure fit'])
plt.title(' Linear curve fit')
plt.xlabel('Temperature[k]')
plt.ylabel('cp')

plt.figure()
plt.plot(temperature, cp, color = 'blue', lw = 3)
plt.plot (temperature, fit_cp2, color = 'green', lw = 3)
plt. legend ( ['Actual data', 'cubical cure fit'])
plt.title(' Cubical curve fit')
plt.xlabel('Temperature[k]')
plt.ylabel('cp')
plt. show()