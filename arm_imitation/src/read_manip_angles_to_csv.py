import rospy
import time
from arm_imitation.msg import msg_joint_angles
import arm_control_utils
import sys

state_file = open("path.csv", "w")
DXL_ID = [1, 2, 3, 8]
my_arm_controller = arm_control_utils.arm_controller(DXL_ID)
my_arm_controller.initialize_motors()

start_time = time.time()

while 1:
    try:
        # Read present position
        state = my_arm_controller.read_state()  # position in steps
        curr_time = time.time()
        deg_state = [( (i/4096.0*360) - 180 ) for i in state]
        str_state = [str(curr_time - start_time)] + [str(i) for i in deg_state]
        state_file.write(",".join(str_state) + "\n")
        time.sleep(0.001)
    except KeyboardInterrupt:
        state_file.close()
        my_arm_controller.stop_motors()
        print("Bye")
        sys.exit()

