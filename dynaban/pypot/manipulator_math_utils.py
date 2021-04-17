import math
import numpy as np
from scipy.optimize import curve_fit
import csv
import sys
import pprint
pp = pprint.PrettyPrinter(indent=4)

def _get_polynomial(t, a3, a2, a1, a0):
    return a3*pow(t, 3) + a2*pow(t, 2) + a1*pow(t, 1) + a0*pow(t, 0)

class manipulator_math_utils:
    def __init__(self, joints):
        self.JOINTS = joints


    def read_csv_file(self, _inp, _with_torque=True):
        # timestamp, angles..n, torque..n
        _timestamps = []
        _angles = []
        if (_with_torque): _torques = []

        for i in range(self.JOINTS):
            _angles.append([])
            if(_with_torque): _torques.append([])

        with open(_inp, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                row_float = list(map(float, row))
                # print(row_float)
                _timestamps.append(row_float[0])
                for i in range(self.JOINTS):
                    _angles[i].append(row_float[1+i])
                    if(_with_torque): _torques[i].append(row_float[1+self.JOINTS+i])

        if(_with_torque): return _timestamps, _angles, _torques
        return _timestamps, _angles


    def angles_to_steps(self, _angles, _transformations):
        for j in range(self.JOINTS):
            _angles[j] = [int((_transformations[j][0]*_angles[j][i] +
                            _transformations[j][1])/360.0*4096) for i in range(len(_angles[j]))]
        return _angles

    def padded_moving_average(self, _angles, _windowsize=5):
        # applying moving average on angles
        for i in range(self.JOINTS):
            
            _angles[i] = (np.convolve(_angles[i], np.ones(_windowsize), 'valid') / _windowsize).tolist()
            _angles[i] = _angles[i] + [_angles[i][-1]]*(_windowsize-1)
            # WINDOWSIZE-1 values are less
            # so we append the last value repeated to fill the difference
        return _angles


    def get_timesplits(self, _timestamps, _spline=1):
        _timesplits = [0]
        startTime = _timestamps[0]

        for i in range(len(_timestamps)):
            if (_timestamps[i] - startTime > _spline):
                _timesplits.append(i)
                startTime = _timestamps[i]

        if (_timesplits[-1] is not len(_timestamps)-1):
            _timesplits.append(len(_timestamps))

        return _timesplits

    def get_coeffs_for_angle(self, _timestamps, _angles, _timesplits):
        _coeffs_angle = []
        for j in range(self.JOINTS):
            _coeffs_angle.append([])  # [[],[]]

        for j in range(self.JOINTS):  # har motor ka angle values liya
            # usme se splined set nikal [0, 10] -> 1
            for i in range(len(_timesplits) - 1):
                splined_timestamps = _timestamps[_timesplits[i]:_timesplits[i+1]]
                splined_timestamps_mapped = [
                    splined_timestamps[k] - splined_timestamps[0] for k in range(len(splined_timestamps))]
                # pp.pprint(splined_timestamps_mapped)

                splined_angles = _angles[j][_timesplits[i]:_timesplits[i+1]]
                # pp.pprint(splined_angles)

                coeffs_splined_angle, cov_splined_angle = curve_fit(
                    _get_polynomial, splined_timestamps_mapped, splined_angles)
                _coeffs_angle[j].append(coeffs_splined_angle.tolist())

        return _coeffs_angle


    def get_coeffs_for_angle_torque(self, _timestamps, _angles, _torques, _timesplits):
        _coeffs_angle = []
        _coeffs_torque = []
        for j in range(self.JOINTS):
            _coeffs_angle.append([])  # [[],[]]
            _coeffs_torque.append([])

        for j in range(self.JOINTS):  # har motor ka angle values liya
            # usme se splined set nikal [0, 10] -> 1
            for i in range(len(_timesplits) - 1):
                splined_timestamps = _timestamps[_timesplits[i]:_timesplits[i+1]]
                splined_timestamps_mapped = [
                    splined_timestamps[k] - splined_timestamps[0] for k in range(len(splined_timestamps))]
                # pp.pprint(splined_timestamps_mapped)

                splined_angles = _angles[j][_timesplits[i]:_timesplits[i+1]]
                splined_torques = _torques[j][_timesplits[i]:_timesplits[i+1]]
                # pp.pprint(splined_angles)

                coeffs_splined_angle, cov_splined_angle = curve_fit(
                    _get_polynomial, splined_timestamps_mapped, splined_angles)
                _coeffs_angle[j].append(coeffs_splined_angle.tolist())
                coeffs_splined_torque, cov_splined_torque = curve_fit(
                    _get_polynomial, splined_timestamps_mapped, splined_torques)
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


    def calculate_coeffs(self, file_name, angle_in_steps=True, transformations=[], with_torque=True, moving_average_windowsize=5, spline=1):
        file_type = file_name.split(".")[-1]
        if(file_type == "csv"):
            data = self.read_csv_file(file_name, with_torque)
        elif(file_type == "json"):
            # to be implemented
            pass
        else:
            print("Wrong file format. Provide csv or json files only")

        if(with_torque): timestamps, angles, torques = data
        else: timestamps, angles = data

        if not angle_in_steps: angles = self.angles_to_steps(angles, transformations)
        angles = self.padded_moving_average(angles, moving_average_windowsize)
        timesplits = self.get_timesplits(timestamps, spline)

        if(with_torque):
            coeffs_angle, coeffs_torque = self.get_coeffs_for_angle_torque(timestamps, angles, torques, timesplits)
            return coeffs_angle, coeffs_torque
        else:
            coeffs_angle = self.get_coeffs_for_angle(timestamps, angles, timesplits)
            return coeffs_angle

if __name__=="__main__":
    my_joints = 3
    my_manipulator_math_utils = manipulator_math_utils(my_joints)
    my_transformations = [[1,0]]*my_joints
    a, b = my_manipulator_math_utils.calculate_coeffs(sys.argv[1])
    pp.pprint(a,b)