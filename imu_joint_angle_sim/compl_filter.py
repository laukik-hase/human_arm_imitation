'''
CSV to list of lists - indices for raw data
0 lower_acce_x, 1 lower_acce_y, 2 lower_acce_z
3 lower_gyro_x, 4 lower_gyro_y, 5 lower_gyro_z
6 upper_acce_x, 7 upper_acce_y, 8 upper_acce_z
9 upper_gyro_x, 10 upper_gyro_y, 11 upper_gyro_z
'''

import time
import math as m
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from csv import reader

file_name = 'dataset/4/imu4.csv'
dt = 0.05
alpha, beta = 0.98, 0.99

pi = m.pi
rad_to_deg = 180.0 / pi

def parse_csv_data(file_name):
    with open(file_name, 'r') as read_obj:
        csv_data = reader(read_obj)
        raw_imu_data = [list(map(float, sublist)) for sublist in list(csv_data)]
        raw_imu_data = list(np.around(np.array(raw_imu_data), 2))
        return raw_imu_data

def compute_acce_angle(Ax, Ay, Az):
    acce_pitch = m.atan2(Ax, m.sqrt(Ay ** 2.0 + Az ** 2.0))
    acce_roll = m.atan2(Ay, m.sqrt(Az ** 2.0 + Ax ** 2.0))

    return [acce_roll, acce_pitch]

def compute_gyro_angle(Gx, Gy, Gz, prev_gyro_roll, prev_gyro_pitch):
    gyro_pitch = prev_gyro_pitch + dt * (Gx + Gy * m.sin(prev_gyro_pitch) * m.tan(prev_gyro_roll) + Gz * m.cos(prev_gyro_pitch) * m.tan(prev_gyro_roll))
    gyro_roll = prev_gyro_roll + dt * (Gy * m.cos(prev_gyro_pitch)  - Gz * m.sin(prev_gyro_pitch))

    return [gyro_roll, gyro_pitch]

def complementary_filter(acce_roll, acce_pitch, gyro_roll, gyro_pitch, prev_roll, prev_pitch):
    filter_roll = alpha * gyro_roll + (1 - alpha) * acce_roll
    filter_pitch = alpha * gyro_pitch + (1 - alpha) * acce_pitch

    final_roll = beta * filter_roll + (1 - beta) * prev_roll
    final_pitch = beta * filter_pitch + (1 - beta) * prev_pitch

    return [final_roll, final_pitch]

if __name__ == "__main__":
    raw_imu_data = parse_csv_data(file_name)
    upper_gyro_angle, lower_gyro_angle = [0]*2, [0]*2
    link_angle, final_angles = [0]*4, []
    first = True

    for value in raw_imu_data:
        Axl, Ayl, Azl = value[0], value[1], value[2]
        Gxl, Gyl, Gzl = value[3], value[4], value[5]

        Axu, Ayu, Azu = value[6], value[7], value[8]
        Gxu, Gyu, Gzu = value[9], value[10], value[11]

        if not first:
            upper_acce_angle = compute_acce_angle(Axu, Ayu, Azu) 
            lower_acce_angle = compute_acce_angle(Axl, Ayl, Azl)
            
            upper_gyro_angle = compute_gyro_angle(Gxu, Gyu, Gzu, upper_gyro_angle[0], upper_gyro_angle[1]) 
            lower_gyro_angle = compute_gyro_angle(Gxl, Gyl, Gzl, lower_gyro_angle[0], lower_gyro_angle[1])

            upper_link_angle = complementary_filter(upper_acce_angle[0], upper_acce_angle[1], upper_gyro_angle[0], upper_gyro_angle[1], link_angle[0], link_angle[1])
            lower_link_angle = complementary_filter(lower_acce_angle[0], lower_acce_angle[1], lower_gyro_angle[0], lower_gyro_angle[1], link_angle[2], link_angle[3])

            link_angle = upper_link_angle + lower_link_angle
        
        else:
            upper_acce_angle = compute_acce_angle(Axu, Ayu, Azu) 
            lower_acce_angle = compute_acce_angle(Axl, Ayl, Azl)

            link_angle = upper_acce_angle + lower_acce_angle
            first = False

        final_angles.append(link_angle[:])

n = np.arange(0.0, len(raw_imu_data) * dt, dt)
labels = ['Upper roll', 'Upper Pitch', 'Lower roll', 'Lower Pitch']

for i in range(len(final_angles[0])):
    plt.subplot(2, 2, i + 1)
    plt.plot(n, [pt[i] * rad_to_deg for pt in final_angles], label=labels[i])
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (Degrees)")
    plt.legend()

plt.tight_layout()
plt.show()

