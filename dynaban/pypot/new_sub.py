from std_msgs.msg import String
import paho.mqtt.client as paho
import time
broker = "broker.hivemq.com"
topic = "fyp/qos"
qos = 1
count = 0
def on_message(client, userdata, message):
    global count,gate
    if count == 1:
        print("###########################")
        msg = message.payload.decode("utf-8") 
        client.unsubscribe(topic)
        print("Hello unsub")
        time.sleep(10)
        client.subscribe(topic, qos)
        count = 2
    elif count == 0:
        count = 1
        msg = message.payload.decode("utf-8") 
        print("Received: ", msg)
    else:
        msg = message.payload.decode("utf-8") 
        print("Received: ", msg)    
def on_connect(client, userdata, flags, rc):
    client.subscribe(topic, qos)


client = paho.Client("client_001", clean_session=False)
client.on_connect = on_connect
client.on_message = on_message
print("Connecting to broker: ", broker)
client.max_queued_messages_set(256)
client.connect(broker, 1883)
client.loop_forever()