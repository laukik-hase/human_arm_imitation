import os
import time
import json
import sys
  
from itertools import *
from math import *

import paho.mqtt.client as paho

broker = "test.mosquitto.org"
topic = "fyp/sensors"
qos = 0
msg_interval = 3

def sim_msg():
    msg_dict = {
        'timestamp': time.time() - start,
        'shoulder': {'roll' : 0.0, 'pitch': 45.0, 'yaw': 90.0},
        'elbow': {'pitch': 45.0}
    }
    msg_json = json.dumps(msg_dict, indent=4)
    return msg_json

# define callback
def on_message(client, userdata, message):
    global run_flag
    msg = message.payload
    print("Received: ",str(msg.decode("utf-8")))

def on_publish(client, userdata, mid):
    client.mid_value = mid
    client.puback_flag = True

def wait_for_pub(client, msgType, period=0.1, wait_time=40, running_loop=False):
    client.running_loop = running_loop
    count = 0
    while True:
        if msgType == "PUBACK":
            if client.on_publish:
                if client.puback_flag:
                    return True
        if not client.running_loop:
            client.loop(0.01)
        time.sleep(period)
        count += 1
        if count > wait_time:
            print("wait_for timeout")
            return False
    return True

def c_publish(client, topic, out_message, qos):
    res, mid = client.publish(topic, out_message, qos, retain = False)
    if res == 0:
        if wait_for_pub(client, "PUBACK", running_loop=True):
            if mid == client.mid_value:
                client.puback_flag = False
            else:
                raise SystemExit("not got correct puback mid so quitting")
        else:
            raise SystemExit("not got puback so quitting")


client = paho.Client("Client - 001")

client.on_publish = on_publish
client.on_message = on_message

client.puback_flag = False
client.mid_value = None

print("Connecting to broker: ", broker)
client.connect(broker, 1883)
client.loop_start()

print("Publishing to", topic)

start = time.time()

while True:
    msg = sim_msg()
    c_publish(client, topic, msg, qos)

client.loop_stop()
client.disconnect()
