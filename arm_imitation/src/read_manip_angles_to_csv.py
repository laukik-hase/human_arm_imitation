import rospy
import time
from arm_imitation.msg import msg_joint_angles
import arm_control_utils
import sys

state_file = open("path.csv", "w")

arm_control_utils.initialize_motors()

while 1:
    try:
        # Read present position
        state = arm_control_utils.read_state()  # position in steps
        deg_state = [i/4096.0*360 for i in state]
        str_state = [str(i) for i in deg_state]
        state_file.write(",".join(str_state) + "\n")
        time.sleep(0.01)
    except KeyboardInterrupt:
        state_file.close()
        arm_control_utils.stop_motors()
        print "Bye"
        sys.exit()

