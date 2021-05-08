from std_msgs.msg import String
import paho.mqtt.client as paho
import time
import Queue as queue
import json

broker = "broker.hivemq.com"
topic = "fyp/qos"
qos = 1
count = 0

def on_message(client, userdata, message):
    msg = message.payload.decode("utf-8")
    q.put(msg)

def on_connect(client, userdata, flags, rc):
    client.subscribe(topic, qos)


client = paho.Client("client_001", clean_session=False)
client.on_connect = on_connect
client.on_message = on_message
print("Connecting to broker: ", broker)
client.max_queued_messages_set(256)
client.connect(broker, 1883)
client.loop_start()
loop_flag = 1
q = queue.Queue()
count = 1
while loop_flag==1:
    if count == 1:
        message = q.get()
        msg = json.loads(message)
        print(msg)
        print("###########################")
        client.unsubscribe(topic)
        print("Hello unsub")
        time.sleep(10)
        client.subscribe(topic, qos)
        count = 2
        
    else:
        message = q.get()
        msg = json.loads(message)
        print(msg)

client.disconnect()
client.loop_stop()
