import rospy
import time
import sys
from arm_imitation.msg import msg_joint_angles

JOINTS = 4

if ( len(sys.argv) == 2 ):
    file_name = sys.argv[1]
else:
    print("Usage: python csv_angle_publisher.py <csv_file_name>")
    exit()

rospy.init_node('csv_angle_publisher', anonymous=True)
pub = rospy.Publisher('/joint_angles', msg_joint_angles, queue_size=1024)

with open(file_name, mode='r') as openfileobject:
    for line in openfileobject:
        str_state = line.split(',')
        state = [float(i) for i in str_state[1:JOINTS+1]]
        print(state)
        pub.publish(state)
        time.sleep(0.05)

