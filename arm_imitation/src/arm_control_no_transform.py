import rospy
import time
from arm_imitation.msg import msg_joint_angles
import arm_control_utils

THRESHOLD = 10

state_file = open("path_error.csv", "w")

def callback(data):
    state = [-1 for i in range(3)]
    # print(data.joint_angles)
    state[0] = int( ( (data.joint_angles[0] ) % 360 ) / 360.0 * 4096 )
    state[1] = int( ( (data.joint_angles[1] ) % 360 ) / 360.0 * 4096 )
    state[2] = int( ( (data.joint_angles[2] ) % 360 ) / 360.0 * 4096 )
    for point in state:
        if (point < 1000 or point > 3100):
            print("invalid")
            print(state)
            return
    arm_control_utils.write_state(state)
    str_state = [str(i) for i in state]
    state_file.write(",".join(str_state) + ",")
    
    current_state = arm_control_utils.read_state()
    str_state = [str(i) for i in current_state]
    state_file.write(",".join(str_state) + "\n")

    time.sleep(0.01)
    
def listener():
    rospy.init_node('arm_controller', anonymous=True)
    rospy.Subscriber("/joint_angles", msg_joint_angles, callback)
    rospy.spin()

arm_control_utils.initialize_motors()
arm_control_utils.enable_state_torque()
# imitate
print("Ready to imitate")
listener()

arm_control_utils.disable_state_torque()
arm_control_utils.stop_motors()