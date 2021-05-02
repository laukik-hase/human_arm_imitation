import rospy
from std_msgs.msg import String
import paho.mqtt.client as paho

broker = "test.mosquitto.org"
topic = "fyp/sensors"
qos = 1

def on_message(client, userdata, message):
    msg = message.payload.decode("utf-8")
    pub.publish(msg)
    print("Received: ", msg)

def on_connect(client, userdata, flags, rc):
    client.subscribe(topic, qos)

rospy.init_node('mqtt_listener', anonymous=True)
pub = rospy.Publisher('/mqtt_data', String, queue_size=1024)
client = paho.Client("client_001")

client.on_connect = on_connect
client.on_message = on_message
print("Connecting to broker: ", broker)
client.connect(broker, 1883)
client.loop_forever()