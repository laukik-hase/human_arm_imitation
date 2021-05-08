import paho.mqtt.client as paho 
import time
import Queue as queue
import json

import pprint
pp = pprint.PrettyPrinter(indent=4)
# import rbdl
# import manip motion


def on_connect(client, userdata, flags, rc):
    client.subscribe(topic, qos)

def on_message(client, userdata, message):
    msg = message.payload.decode("utf-8")
    q.put(msg)
#     print("Received: ", msg)

broker = "broker.hivemq.com"
topic = "fyp/demo"
qos = 0

client = paho.Client("client_001")     
client.on_connect=on_connect
client.on_message = on_message        
client.connect(broker)      
client.loop_start()
loop_flag=1
q = queue.Queue()