import os
import time
import json
import sys

from itertools import *
from math import *

import paho.mqtt.client as paho

broker = "test.mosquitto.org"
topic = "fyp/sensors"
qos = 1


def on_message(client, userdata, message):
    msg = str(message.payload.decode("utf-8"))
    print("Received: ", msg)


def on_connect(client, userdata, flags, rc):
    client.subscribe(topic, qos)


client = paho.Client("client_001")

client.on_connect = on_connect
client.on_message = on_message

print("Connecting to broker: ", broker)
client.connect(broker, 1883)
client.loop_forever()
