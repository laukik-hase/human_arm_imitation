import rospy
import time
from arm_imitation.msg import msg_joint_angles
import arm_control_utils

record_error = False
offset = [180, 180, 180, 180]
# offset = [180 180 90] # niryo
DXL_ID = [3, 4]
my_arm_controller = arm_control_utils.arm_controller(DXL_ID)

if record_error:
    state_file = open("path_error.csv", "w")

def callback(data):
    # state = [ int( ( (data.joint_angles[i] + offset[i]) % 360 ) / 360.0 * 4096 ) for i in range(my_arm_controller.JOINTS)]
    # print(state)
    print("test")
    state = [int( ( (data.joint_angles[0] + offset[0]) % 360 ) / 360.0 * 4096 ), int( ( (data.joint_angles[1] + offset[1]) % 360 ) / 360.0 * 4096 )]
    print(state)
    
    # print(data.joint_angles)

    # for point in state:
    #     if (point < 1000 or point > 3100):
    #         print("invalid")
    #         print(state)
    #         return
    my_arm_controller.write_state(state)
    
    if record_error:
        str_state = [str(i) for i in state]
        state_file.write(",".join(str_state) + ",")
        
        current_state = my_arm_controller.read_state()
        str_state = [str(i) for i in current_state]
        state_file.write(",".join(str_state) + "\n")

    time.sleep(0.01)
    
def listener():
    rospy.init_node('arm_controller', anonymous=True)
    rospy.Subscriber("/joint_angles", msg_joint_angles, callback)
    rospy.spin()

my_arm_controller.initialize_motors()
my_arm_controller.enable_state_torque()
# imitate
print("Ready to imitate")
listener()

my_arm_controller.disable_state_torque()
my_arm_controller.stop_motors()