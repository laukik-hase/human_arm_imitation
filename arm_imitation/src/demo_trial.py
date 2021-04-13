import rospy
import time
import sys
from arm_imitation.msg import msg_joint_angles
import arm_control_utils

offset = [180, 180, 180, 180]
# offset = [180 180 90] # niryo
DXL_ID = [2, 4]
my_arm_controller = arm_control_utils.arm_controller(DXL_ID, devicename='/dev/ttyUSB2')



#reading csv file

    
# def listener():
#     rospy.init_node('arm_controller', anonymous=True)
#     rospy.Subscriber("/joint_angles", msg_joint_angles, callback)
#     rospy.spin()

my_arm_controller.initialize_motors()
my_arm_controller.enable_state_torque()
# imitate
my_arm_controller.enable_torque(1)
my_arm_controller.enable_torque(2)
csv_file = open("compare_torque_data.csv", "w")
if ( len(sys.argv) == 2 ):
    file_name = sys.argv[1]
else:
    print("Usage: python csv_angle_publisher.py <csv_file_name>")
    exit()

with open(file_name, mode='r') as openfileobject:
    for line in openfileobject:
        str_state = line.split(',')
        print(str_state)
        state = [int( ((float(str_state[0])+180) % 360 ) / 360.0 * 4096 ), int( ( (float(str_state[1]) + 180) % 360 ) / 360.0 * 4096 )]
        print(state)
        my_arm_controller.write_state(state)
        # time.sleep(0.03)
        torque_state = my_arm_controller.read_torque()
        str_torque_state = [str(t) for t in torque_state]
        csv_file.write(",".join(str_torque_state) + "\n")

my_arm_controller.disable_state_torque()
my_arm_controller.stop_motors()