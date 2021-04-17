import math
import numpy as np 
from scipy.optimize import curve_fit
import csv
import sys
import pprint

pp = pprint.PrettyPrinter(width=41, compact=True)
JOINTS = 3
COLS = 1 + 2*JOINTS
WINDOWSIZE = 5
SPLINE = 1

def read_csv_file(_inp):
    # timestamp, angles..n, torque..n
    _timestamps = []
    _angles = []
    _torques = []
    for i in range(JOINTS):
        _angles.append([])
        _torques.append([])

    with open(_inp, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            row_float = [float(val) for val in row]
            # print(row_float)
            _timestamps.append(row_float[0])
            for i in range(JOINTS):
                _angles[i].append(row_float[1+i])
                _torques[i].append(row_float[1+JOINTS+i])

    return _timestamps, _angles, _torques
        
def moving_average(x, w):
    return (np.convolve(x, np.ones(w), 'valid') / w)

def resized_moving_average(_angles, _windowsize):
    # applying moving average on angles
    
    for i in range(JOINTS):
        _angles[i] = moving_average(_angles[i], _windowsize).tolist()
        _angles[i] = _angles[i] + [ _angles[i][-1] ]*(_windowsize-1)
        # WINDOWSIZE-1 values are less
        # so we append the last value repeated to fill the difference
    return _angles

def get_timesplits(_timestamps, _spline):
    _timesplits = [0]
    startTime = _timestamps[0]

    for i in range(len(_timestamps)):
        if (_timestamps[i] - startTime > _spline):
            _timesplits.append(i)
            startTime = _timestamps[i]

    if (_timesplits[-1] is not len(_timestamps)-1):
        _timesplits.append(len(_timestamps))

    return _timesplits

def get_polynomial(t, a3, a2, a1, a0):
    return a3*pow(t,3) + a2*pow(t,2) + a1*pow(t,1) + a0*pow(t,0)

def get_coeffs(_timestamps, _angles, _torques, _timesplits):
    _coeffs_angle = []
    _coeffs_torque = []
    for i in range(JOINTS):
        _coeffs_angle.append([]) #[[],[]]
        _coeffs_torque.append([])

    for j in range(JOINTS): # har motor ka angle values liya
        for i in range(len(_timesplits) - 1): # usme se splined set nikal [0, 10] -> 1
            splined_timestamps = _timestamps[_timesplits[i]:_timesplits[i+1]]
            splined_timestamps_mapped = [ splined_timestamps[k] - splined_timestamps[0] for k in range(len(splined_timestamps))]
            # pp.pprint(splined_timestamps_mapped)
        
            splined_angles = _angles[j][_timesplits[i]:_timesplits[i+1]]
            splined_torques = _torques[j][_timesplits[i]:_timesplits[i+1]]
            # pp.pprint(splined_angles)
            
            coeffs_splined_angle, cov_splined_angle= curve_fit(get_polynomial, splined_timestamps_mapped, splined_angles)
            _coeffs_angle[j].append(coeffs_splined_angle.tolist())
            coeffs_splined_torque, cov_splined_torque= curve_fit(get_polynomial, splined_timestamps_mapped, splined_torques)
            _coeffs_torque[j].append(coeffs_splined_torque.tolist())

    return _coeffs_angle, _coeffs_torque

    # [
    #     motor1
    #     [
    #         first 1s values
    #         [

    #         ],ar
    #         next
    #         [

    #         ]
    #     ]
    #     motor2
    #     [

    #     ]
    # ]

# main Program

file_name = sys.argv[1]
timestamps, angles, torques = read_csv_file(file_name)
angles = resized_moving_average(angles, WINDOWSIZE)
timesplits = get_timesplits(timestamps, SPLINE)
coeffs_angle, coeffs_torque = get_coeffs(timestamps, angles, torques, timesplits)
pp.pprint(coeffs_angle)