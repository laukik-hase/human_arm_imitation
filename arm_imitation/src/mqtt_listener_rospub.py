import rospy
# from std_msgs.msg import String
import paho.mqtt.client as paho
import time 
import json
from arm_imitation.msg import msg_joint_angles
broker = "test.mosquitto.org"
topic = "fyp/sensors"
qos = 0


def on_message(client, userdata, message):
    angles = []
    msg = message.payload.decode("utf-8")
    msg = json.loads(msg)
    angles.append(msg['shoulder']['pitch'])
    angles.append(msg['shoulder']['roll'])
    angles.append(msg['shoulder']['yaw'])
    angles.append(msg['elbow']['pitch'])
    print(angles)
    pub.publish(angles)
    time.sleep(0.05)
    # print("Received: ", msg)

def on_connect(client, userdata, flags, rc):
    client.subscribe(topic, qos)

rospy.init_node('mqtt_listener', anonymous=True)
pub = rospy.Publisher('/joint_angles', msg_joint_angles, queue_size=1024)
client = paho.Client("client_001", clean_session=True)

client.on_connect = on_connect
client.on_message = on_message
print("Connecting to broker: ", broker)
client.max_queued_messages_set(256)
client.connect(broker, 1883)
client.loop_forever()
