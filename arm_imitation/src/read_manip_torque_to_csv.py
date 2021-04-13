import rospy
import time
from arm_imitation.msg import msg_joint_angles
import arm_control_utils
import sys

state_file = open("torque_data.csv", "w")
DXL_ID = [2, 4]
my_arm_controller = arm_control_utils.arm_controller(DXL_ID, devicename='/dev/ttyUSB2')
my_arm_controller.initialize_motors()

start_time = time.time()
my_arm_controller.enable_state_torque()
while 1:
    try:
        # Read present position
        state = my_arm_controller.read_torque()  # position in steps
        curr_time = time.time()
        print(state)
#         deg_state = [( (i/4096.0*360) - 180 ) for i in state]
#        str_state = [str(curr_time - start_time)] + [str(i) for i in deg_state]
        state_file.write(",".join(str(state)) + "\n")
        time.sleep(0.001)
    except KeyboardInterrupt:
        state_file.close()
        my_arm_controller.stop_motors()
        print("Bye")
        sys.exit()

