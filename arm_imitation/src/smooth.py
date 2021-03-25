import rospy
import time
import numpy as np
from arm_imitation.msg import msg_joint_angles
import arm_control_utils

record_error = False
offset = [180, 180, 180, 180]
# offset = [180 180 90] # niryo
DXL_ID = [1, 2, 3, 8]
my_arm_controller = arm_control_utils.arm_controller(DXL_ID)


my_arm_controller.initialize_motors()
my_arm_controller.enable_state_torque()
# imitate
print("Ready to imitate")
angles = np.linspace(0, 2*np.pi, 100)
for angle in angles:
    x = [2048, 2048, 2048, int( (35*np.sin(angle) + 35 + 180) / 360.0 * 4096 ) ]
    print(x)
    my_arm_controller.write_state(x)
    time.sleep(0.03)


my_arm_controller.disable_state_torque()
my_arm_controller.stop_motors()