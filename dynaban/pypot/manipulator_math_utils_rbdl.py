import math
import numpy as np
from scipy.optimize import curve_fit
import csv
import sys
import pprint
import rbdlpy

pp = pprint.PrettyPrinter(indent=4)

def _get_polynomial(t, a3, a2, a1, a0):
    return a3*pow(t, 3) + a2*pow(t, 2) + a1*pow(t, 1) + a0*pow(t, 0)

class manipulator_math_utils:
    def __init__(self, joints, with_torque=True):
        self.JOINTS = joints
        self.WITH_TORQUE = with_torque
        self.start_angles = []

    def read_csv_file(self, _csv_file, _urdf_file=""):
        # timestamp, angles..n, torque..n
        _data = np.genfromtxt(_csv_file, delimiter=',')
        _timestamps = _data[:,0]
        _angles = _data[:,1:]
        if(_urdf_file):
            rbdl_obj = rbdlpy.rbdlpy(_urdf_file)
            _torques = rbdl_obj.inverse_dynamics(_timestamps, _angles)
            _torques = np.transpose(_torques).tolist()

        _angles = np.transpose(_angles).tolist()
        _timestamps = _timestamps.tolist()

        self.start_angles = [_angles[i][0] for i in range(self.JOINTS)]
        print(self.start_angles)
        if(_urdf_file != ""): return _timestamps, _angles, _torques
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
        startTime = 0

        for i in range(len(_timestamps)):
            if (_timestamps[i] - startTime > _spline):
                _timesplits.append(i)
                startTime = startTime + 1

        # print(_timesplits)
        return _timesplits

    def pad_data_angle_torque(self, _timestamps, _angles, _torques, _timesplits):
        if (_timesplits[-1] is not len(_timestamps)-1):
            padding_delta_time = _timestamps[_timesplits[-1] + 1] - _timestamps[_timesplits[-1]]
            no_of_padding  = int( ( math.ceil(_timestamps[-1]) - _timestamps[-1] ) / padding_delta_time )
            _timestamps.extend(np.arange(_timestamps[-1] + padding_delta_time, math.ceil(_timestamps[-1]), padding_delta_time))
            for j in range(self.JOINTS):
                _angles[j].extend([_angles[j][-1]] * no_of_padding)
                _torques[j].extend([_torques[j][-1]] * no_of_padding)
            _timesplits.append(len(_timestamps)-1)
        return _timestamps, _angles, _torques, _timesplits

    def pad_data_angle(self, _timestamps, _angles, _timesplits):
        if (_timesplits[-1] is not len(_timestamps)-1):
            padding_delta_time = _timestamps[_timesplits[-1] + 1] - _timestamps[_timesplits[-1]]
            no_of_padding  = int( ( math.ceil(_timestamps[-1]) - _timestamps[-1] ) / padding_delta_time )
            _timestamps.extend(np.arange(_timestamps[-1] + padding_delta_time, math.ceil(_timestamps[-1]), padding_delta_time))
            for j in range(self.JOINTS):
                _angles[j].extend([_angles[j][-1]] * no_of_padding)
            _timesplits.append(len(_timestamps)-1)
        return _timestamps, _angles, _timesplits

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
        #         first 1 second values
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


    def calculate_coeffs(self, _csv_file, transformations=[], _urdf_file="", moving_average_windowsize=5, spline=1):
        data = self.read_csv_file(_csv_file, _urdf_file)

        if(_urdf_file): timestamps, angles, torques = data
        else: timestamps, angles = data

        if (transformations): angles = self.angles_to_steps(angles, transformations)
        angles = self.padded_moving_average(angles, moving_average_windowsize)
        timesplits = self.get_timesplits(timestamps, spline)

        if (_urdf_file): timestamps, angles, torques, timesplits = self.pad_data_angle_torque(timestamps, angles, torques, timesplits)
        else: timestamps, angles, timesplits = self.pad_data_angle(timestamps, angles, timesplits)
        # pp.pprint(timestamps)
        if(_urdf_file):
            coeffs_angle, coeffs_torque = self.get_coeffs_for_angle_torque(timestamps, angles, torques, timesplits)
            return coeffs_angle, coeffs_torque
        else:
            coeffs_angle = self.get_coeffs_for_angle(timestamps, angles, timesplits)
            return coeffs_angle

if __name__=="__main__":
    my_joints = 4
    my_manipulator_math_utils = manipulator_math_utils(my_joints)
    my_transformations = [[-1,180],[1,180],[-1,180],[-1,180]]
    my_urdf_file = ""
    if len(sys.argv) == 3:
        my_urdf_file = sys.argv[2]

    coeffs = my_manipulator_math_utils.calculate_coeffs(sys.argv[1], my_transformations, my_urdf_file)
    pp.pprint(coeffs)