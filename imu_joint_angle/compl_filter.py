'''
CSV to list of lists - indices for raw data
0 upper_acce_x, 1 upper_acce_y, 2 upper_acce_z
3 upper_gyro_x, 4 upper_gyro_y, 5 upper_gyro_z
6 lower_acce_x, 7 lower_acce_y, 8 lower_acce_z
9 lower_gyro_x, 10 lower_gyro_y, 11 lower_gyro_z
'''

import time
import math as m
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from csv import reader

file_name = 'imu.csv'
dt, alpha = 0.05, 0.98

pi = m.pi
rad_to_deg = 180.0 / pi
n = np.arange(0.0, 41.90, 0.05)

def parse_csv_data(file_name):
    with open(file_name, 'r') as read_obj:
        csv_data = reader(read_obj)
        raw_imu_data = [list(map(float, sublist)) for sublist in list(csv_data)]
        raw_imu_data = list(np.around(np.array(raw_imu_data), 2))
        return raw_imu_data

def compute_acce_angle(Ax, Ay, Az):
    acce_roll = m.atan2(Ax, m.sqrt(Ay ** 2.0 + Az ** 2.0))
    acce_pitch = m.atan2(Ay, m.sqrt(Ax ** 2.0 + Az ** 2.0))
    return [acce_roll, acce_pitch]

def compute_gyro_angle(Gx, Gy, Gz, prev_roll, prev_pitch):
    Gx /= (131 * rad_to_deg) 
    Gy /= (131 * rad_to_deg)
    Gz /= (131 * rad_to_deg)

    gyro_roll = prev_roll + dt * (Gx + Gy * m.sin(prev_roll) * m.tan(prev_pitch) + Gz * m.cos(prev_roll) * m.tan(prev_pitch))
    gyro_pitch = prev_pitch + dt * (Gy * m.cos(prev_roll) - Gz * m.sin(prev_roll))
    return [gyro_roll, gyro_pitch]

def complementary_filter(acce_roll, acce_pitch, gyro_roll, gyro_pitch):
    filter_roll = (alpha * gyro_roll + (1 - alpha) * acce_roll)
    filter_pitch = (alpha * gyro_pitch + (1 - alpha) * acce_pitch)
    return [filter_roll, filter_pitch]
    
if __name__ == "__main__":
    raw_imu_data = parse_csv_data(file_name)
    acce_angle, gyro_angle = [0]*4, [0]*4
    link_angle, final_angles = [0]*4, []
    first = True

    for value in raw_imu_data:
        Axu, Ayu, Azu = value[0], value[1], value[2]
        Gxu, Gyu, Gzu = value[3], value[4], value[5]

        Axl, Ayl, Azl = value[6], value[7], value[8]
        Gxl, Gyl, Gzl = value[9], value[10], value[11]

        if not first:
            upper_acce_angle = compute_acce_angle(Axu, Ayu, Azu) 
            lower_acce_angle = compute_acce_angle(Axl, Ayl, Azl)
            
            upper_gyro_angle = compute_gyro_angle(Gxu, Gyu, Gzu, link_angle[0], link_angle[1]) 
            lower_gyro_angle = compute_gyro_angle(Gxl, Gyl, Gzl, link_angle[2], link_angle[3])

            upper_link_angle = complementary_filter(upper_acce_angle[0], upper_acce_angle[1], upper_gyro_angle[0], upper_gyro_angle[1])
            lower_link_angle = complementary_filter(lower_acce_angle[0], lower_acce_angle[1], lower_gyro_angle[0], lower_gyro_angle[1])

            link_angle = upper_link_angle + lower_link_angle
        else:
            upper_acce_angle = compute_acce_angle(Axu, Ayu, Azu) 
            lower_acce_angle = compute_acce_angle(Axl, Ayl, Azl)

            link_angle = upper_acce_angle + lower_acce_angle
            first = False

        final_angles.append(link_angle[:])

labels = ['Upper Roll', 'Upper Pitch', 'Lower Roll', 'Lower Pitch']

for i in range(len(final_angles[0])):
    plt.plot(n, [pt[i] * rad_to_deg for pt in final_angles], label=labels[i])
    
plt.xlabel("Time (s)")
plt.ylabel("Angle (Degrees)")
plt.legend()
plt.show()


    


        

