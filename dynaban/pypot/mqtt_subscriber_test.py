import rospy
from std_msgs.msg import String
import paho.mqtt.client as paho
import Queue as queue
import json

broker = "broker.hivemq.com"
topic = "fyp/demo"
qos = 2
q = queue.Queue()
def on_message(client, userdata, message):
    msg = message.payload.decode("utf-8")
    # pub.publish(msg)
    # print("Received: ", msg)
    q.put(msg)

def on_connect(client, userdata, flags, rc):
    client.subscribe(topic, qos)


# rospy.init_node('mqtt_listener', anonymous=True)
# pub = rospy.Publisher('/mqtt_data', String, queue_size=1024)
client = paho.Client("client_001", clean_session=False)

client.on_connect = on_connect
client.on_message = on_message
print("Connecting to broker: ", broker)
client.max_queued_messages_set(256)
client.connect(broker, 1883)
# client.loop_forever()
client.loop_start()
prev_msg = {'shoulder':{'pitch':0}}
while True:
    # time.sleep(1)
    message = q.get()
    msg = json.loads(message)
    print(msg['timestamp'], " ", msg['shoulder']['pitch'])
    if abs(msg['shoulder']['pitch'] - prev_msg['shoulder']['pitch']) > 1:
        print("##############################################")
    prev_msg = msg
client.loop_stop()
client.disconnect()