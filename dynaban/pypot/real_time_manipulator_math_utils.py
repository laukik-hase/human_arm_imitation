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

    

    def angles_to_steps(self, _angles, _transformations):
        for j in range(self.JOINTS):
            _angles[j] = [int((_transformations[j][0]*_angles[j][i] +
                               _transformations[j][1])/360.0*4096) for i in range(len(_angles[j]))]
        return _angles

    def real_time_moving_average(self, _angles, _windowsize=5):
    # applying moving average on angles
        for i in range(self.JOINTS):

            _angles[i] = (np.convolve(_angles[i], np.ones(
                _windowsize), 'valid') / _windowsize).tolist()
            # WINDOWSIZE-1 values are less
            # so we append the last value repeated to fill the difference
        return _angles

    def calculate_coefficients_angles(self, _timestamps, _angles):
        # timestamps between 0 and 1
        # angles in steps

        _coeffs_angle = []
        for j in range(self.JOINTS):
            _coeffs_angle.append([])  # [[],[]]

        for j in range(self.JOINTS):  # har motor ka angle values liya
            # usme se splined set nikal [0, 10] -> 1
            coeffs_splined_angle, cov_splined_angle = curve_fit(
                _get_polynomial, _timestamps, _angles[j])
            _coeffs_angle[j] = coeffs_splined_angle.tolist()
        return _coeffs_angle
    
    def calculate_coefficients_angles_torques(self, _timestamps, _angles, _torques):
        # timestamps between 0 and 1
        # angles in steps

        # [
        #   [ 1,2,3,4 ],
        #   ... 4
        # ]
        _coeffs_angle = []
        _coeffs_torque = []
        for j in range(self.JOINTS):
            _coeffs_angle.append([])  # [[],[]]
            _coeffs_torque.append([])

        for j in range(self.JOINTS):  # har motor ka angle values liya
            # usme se splined set nikal [0, 10] -> 1
            coeffs_splined_angle, cov_splined_angle = curve_fit(
                _get_polynomial, _timestamps, _angles)
            _coeffs_angle[j] = coeffs_splined_angle.tolist()
            coeffs_splined_torque, cov_splined_torque = curve_fit(
                _get_polynomial, _timestamps, _torques)
            _coeffs_torque[j] = coeffs_splined_torque.tolist()
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

if __name__ == "__main__":
    my_joints = 2
    my_manipulator_math_utils = manipulator_math_utils(my_joints)
    my_transformations = [[1, 0]]*my_joints
    a, b = my_manipulator_math_utils.calculate_coefficien(sys.argv[1])
    # pp.pprint(b)
