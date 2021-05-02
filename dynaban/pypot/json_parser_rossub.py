import rospy
from std_msgs.msg import String
import json
cnt = 1
spline = 1
timestamps = []
angles = []
for i in range(4):
    angles.append([])
def callback(data):
    msg = json.loads(data.data)
    print(msg['shoulder']['pitch'])
    timestamps.append(msg['timestamp'])
    angles[0].append(msg['shoulder']['pitch'])
    angles[1].append(msg['shoulder']['roll'])
    angles[2].append(msg['shoulder']['yaw'])
    angles[3].append(msg['elbow']['pitch'])
    if msg['timestamp'] > (timestamps[0]+cnt)*spline:
        print(cnt , msg['timestamp'], angles[0])
        cnt = cnt + 1
def listener():

    rospy.init_node('json_parser_rossub', anonymous=True)
    rospy.Subscriber('/mqtt_data', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()