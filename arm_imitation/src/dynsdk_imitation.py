import paho.mqtt.client as paho 
import time
import Queue as queue
import json
import pprint
import time
import threading
import sys
import arm_control_utils
import manipulator_math_utils
import signal

def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    my_arm_controller.disable_state_torque()
    my_arm_controller.stop_motors()
    client.loop_stop()
    client.disconnect()
    sys.exit(0)

def on_message(client, userdata, message):
    msg = message.payload.decode("utf-8")
    q.put(msg)
    # pp.pprint(msg)
    # print("on message: ", time.time() - will)

def on_connect(client, userdata, flags, rc):
    client.subscribe(topic, qos)

def go_to_start_pos(init_angle):
    for joints in range(len(init_angle)):
        start_angle = init_angle[joints]
        print("start angle",start_angle)
        current_pos = my_arm_controller.get_present_position(DXL_ID[joints])
        diff = abs(current_pos-start_angle)
        print("diff", diff)
        for i in range(0, diff, 10):
            if (current_pos > start_angle):
                my_arm_controller.set_goal_position(DXL_ID[joints], current_pos - i)
                time.sleep(0.05) 
            else:
                my_arm_controller.set_goal_position(DXL_ID[joints], current_pos + i)
                time.sleep(0.05)

def angle_to_step(_angle, _transformation):
    _step = []
    for i in range(len(_angle)):
        _step.append(int((_angle[i] * _transformation[i][0] + _transformation[i][1])/360.0 * 4096))
    return _step

signal.signal(signal.SIGINT, signal_handler)

DXL_ID = [3]
JOINTS = len(DXL_ID)
TRANSFORMATION = [[-1,180]]

# my_arm_controller = arm_control_utils.arm_controller(DXL_ID,devicename='/dev/ttyUSB0', baudrate=1000000)
# my_arm_controller.initialize_motors()
# my_arm_controller.enable_state_torque()


pp = pprint.PrettyPrinter(indent=4)

broker = "broker.hivemq.com"
topic = "fyp/test"
qos = 2

client = paho.Client("client_001", clean_session=True)
client.on_connect = on_connect
client.on_message = on_message
print("Connecting to broker: ", broker)
client.max_queued_messages_set(256)
client.connect(broker, 1883)
client.loop_start()
q = queue.Queue()

prev_steps = []
STEP_DIFF = 32
at_init_pos = False

while True:
    message = q.get()
    msg = json.loads(message)
    angles =[ msg['shoulder']['pitch']]
    steps = angle_to_step(angles, TRANSFORMATION)
    print(steps)
    if not at_init_pos:
        # go_to_start_pos(steps)
        at_init_pos = True
    else:
        n_max = -1
        for i in range(JOINTS):
            if steps[i] - prev_steps[i] > STEP_DIFF:
                n_max = max(n_max, (steps[i] - prev_steps[i])%STEP_DIFF)

        for i in range()
        # my_arm_controller.write_state(steps)
        
        # if diff between steps in more than n
        step_diff = 32
        #  konse angle ka stepdiff sabse jyada he
        # uska stepdiff / 32 -> n
        # n parts me sab angles ko tod


        time.sleep(0.05)
    prev_steps = steps