import time
import numpy
import json

import utils
import pypot.robot
import pypot.dynamixel
import motorTrajectoryManager

listOfTraj =    [
                    [0.0029282637700648157, 532.49287882689293, 29.07849099701108, -1058.1470413492355, 459.3664329672057], 
                    [169.9973127875532, 1.2118904739507608, -859.49525560910968, 109.93882674890278, 489.17556618589202], 
                    [-0.0029282637700933502, -532.49287882689202, -29.078490997017791, 1058.1470413492527, -459.36643296722087], 
                    [-169.99731278755326, -1.2118904739506096, 859.49525560910888, -109.93882674889758, -489.17556618590021]
                ]
listOfTorques = []

robot = pypot.robot.from_json("robot_config.json")
params = utils.Parameters(robot, 0, 50)
trajMan =  motorTrajectoryManager.MotorTrajectoryManager()
trajMan.createMotorTrajectory(params.robot.motors[0], listOfTraj, listOfTorques, [5000, 5000, 5000, 5000], mode=2, repeat=False, delay=0.1)
trajMan.start()
while(True):
    try: 
        trajMan.tick()
        time.sleep(0.1)
    except KeyboardInterrupt:
        exit