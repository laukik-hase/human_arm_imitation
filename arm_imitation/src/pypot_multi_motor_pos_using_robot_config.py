# Code to receive angles with timestamp from csv
# and move the manipulator based on those angles
# using pypot library robot package and robot_config.json

import time
import sys
# pypot imports
import pypot.robot

robot = pypot.robot.from_json("robot_config.json")
print(robot.motors)



for m in robot.motors:
    m.complient = False
    m.goal_position = 90

time.sleep(0.5)
# robot.goto_position(state,1000)
