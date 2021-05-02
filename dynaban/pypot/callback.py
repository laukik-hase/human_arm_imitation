import paho.mqtt.client as paho 
import time
import Queue as queue
import json

cnt = 1
spline = 1
timestamps = []
angles = []
first_value
for i in range(4):
    angles.append([])

def on_connect(client, userdata, flags, rc):
    client.subscribe(topic, qos)

def on_message(client, userdata, message):
    msg = message.payload.decode("utf-8")
    q.put(msg)
    print("Received: ", msg)

broker = "test.mosquitto.org"
topic = "fyp/sensors"
qos = 1

client = paho.Client("client_001")     
client.on_connect=on_connect
client.on_message = on_message        
client.connect(broker)      
client.loop_start()
loop_flag=1
q = queue.Queue()
while loop_flag==1:
    message = q.get()
    msg = json.loads(message)
    timestamps.append(msg['timestamp'])
    angles[0].append(msg['shoulder']['pitch'])
    angles[1].append(msg['shoulder']['roll'])
    angles[2].append(msg['shoulder']['yaw'])
    angles[3].append(msg['elbow']['pitch'])
    if msg['timestamp'] > (timestamps[0]+cnt)*spline:
        print(cnt , msg['timestamp'], angles[0])
        cnt = cnt + 1

client.disconnect()
client.loop_stop()