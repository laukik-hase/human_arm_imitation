# Code to receive angles with timestamp from csv
# and move the manipulator based on those angles
# using pypot library robot package and robot_config.json

import time
import sys
# pypot imports
import pypot.dynamixel
import pypot.robot

robot = pypot.robot.from_json("robot_config.json")
# print(robot.motors)

if ( len(sys.argv) == 2 ):
    file_name = sys.argv[1]
else:
    print("Usage: python csv_angle_publisher.py <csv_file_name>")
    exit()

at_initial_pos = False
ID_LIST = [1, 2]
ID_SIZE = len(ID_LIST)

# print(robot.motors.name)

with open(file_name, mode='r') as openfileobject:
    for line in openfileobject:
        str_state = line.split(',')
        state = [float(str_state[i+1]) for i in range(ID_SIZE)]
        # print(state)
        state_dict = dict(zip(['shoulder_yaw', 'shoulder_roll'], state))
        print(state_dict)
        if not at_initial_pos:
            robot.goto_position(state_dict, 3)
            print("[+] Moving to initial pos")
            time.sleep(2)
            at_initial_pos = True
            print("[+] At initial pos")
        else:
            robot.goto_position(state_dict, 3)
