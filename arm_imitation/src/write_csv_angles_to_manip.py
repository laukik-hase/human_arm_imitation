import rospy
import time
from arm_imitation.msg import msg_joint_angles
import arm_control_utils

rospy.init_node('csv_angle_publisher', anonymous=True)
pub = rospy.Publisher('/joint_angles', msg_joint_angles, queue_size=1024)

with open('path.csv', mode='r') as openfileobject:
    for line in openfileobject:
        str_state = line.split(',')
        state = [float(i) for i in str_state]
        print(state)
        pub.publish(state)
        time.sleep(0.01)

