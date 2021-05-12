import os
import time
import json
import sys
import numpy as np  
from itertools import *
from math import *

from numpy.lib.function_base import angle


import paho.mqtt.client as paho

def c_publish(client, topic, out_message, qos):
    res, mid = client.publish(topic, out_message, qos, retain = False)

broker = "test.mosquitto.org"
topic = "fyp/test"
qos = 0
msg_interval = 3
flag = True
pitch,roll,yaw, epitch =0,0,0,0 
demo_time = 0
DELAY = 0.05

def on_message(client, userdata, message):
    global run_flag
    msg = message.payload
    print("Received: ",str(msg.decode("utf-8")))

def on_publish(client, userdata, mid):
    client.mid_value = mid
    client.puback_flag = True


client = paho.Client("Client - 001", clean_session=True)

client.on_publish = on_publish
client.on_message = on_message

client.puback_flag = False
client.mid_value = None

print("Connecting to broker: ", broker)
client.connect(broker, 1883)

client.loop_start()

print("Publishing to", topic)

csv_file = sys.argv[1]
data = np.genfromtxt(csv_file, delimiter=',')
angles = data[:,1:]
timestamps = data[:,0]

for i in range(len(timestamps)):

    msg_dict = {
        'timestamp': timestamps[i],
        'shoulder': {'roll' : angles[i][1], 'pitch': angles[i][0], 'yaw': angles[i][2]},
        'elbow': {'pitch': angles[i][3]}
    }

    msg_json = json.dumps(msg_dict, indent=4)

    c_publish(client, topic, msg_json, qos)
    time.sleep(DELAY)


client.loop_stop()
client.disconnect()
