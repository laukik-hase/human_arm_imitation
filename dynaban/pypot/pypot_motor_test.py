import time
import numpy
import json

# pypot imports
import pypot.robot
import pypot.dynamixel

robot = pypot.robot.from_json("robot_config.json")
print(robot.motors)

# for m in robot.motors:
#     print(m.present_position)
