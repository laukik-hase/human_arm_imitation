from tokenize import String
import paho.mqtt.client as paho 
import time
import Queue as queue
import json
import pprint
import time
import threading
import sys
import arm_control_utils
import signal
import numpy as np

def signal_handler(signal, frame):
    if RUN_MOTORS:
        print("Stopping dynamixel...")
        my_arm_controller.disable_state_torque()
        for id in UNUSED_ID:
            my_arm_controller.disable_torque(id)
        my_arm_controller.disable_torque(GRIPPER_ID)
        my_arm_controller.stop_motors()
    print("Stopping MQTT...")
    client.loop_stop()
    client.disconnect()
    np.savetxt("delay_wo_motor.csv",delay_data)
    print("Goodbye!!!")
    sys.exit(0)

def on_message(client, userdata, message):
    msg = message.payload.decode("utf-8")
    q.put(msg)
    # pp.pprint(msg)
    # print("on message: ", time.time() - will)

def on_connect(client, userdata, flags, rc):
    client.subscribe(topic, qos)

def gripper_move(_gripper_state):
    if _gripper_state:
        print("Closing gripper...")
        start = my_arm_controller.get_present_position(GRIPPER_ID)
        print(start)
        cur = int(start)
        while(cur > CLOSE_LIMIT):
            cur = cur - 1
            my_arm_controller.set_goal_position(GRIPPER_ID, cur)
            time.sleep(0.01)
            print(cur)
        print("Gripper closed")
        print(my_arm_controller.get_present_position(GRIPPER_ID))

    else:
        print("Opening gripper...")
        start = my_arm_controller.get_present_position(GRIPPER_ID)
        print(start)
        cur = int(start)
        while(cur < OPEN_LIMIT):
            cur = cur + 1
            my_arm_controller.set_goal_position(GRIPPER_ID, cur)
            time.sleep(0.01)
        print("Gripper opened")
        print(my_arm_controller.get_present_position(GRIPPER_ID))

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

pp = pprint.PrettyPrinter(indent=4)

broker = "test.mosquitto.org"
topic = "fyp/test"
qos = 0

DXL_ID = [1,4,3]
UNUSED_ID = [2]
GRIPPER_ID = 5
OPEN_LIMIT = 1023
CLOSE_LIMIT = 1023 - 200 # 170 steps stands for 50 degrees
JOINTS = len(DXL_ID)
TRANSFORMATION = [[-1,90], [1,180],[-1,180]] # left arm
# TRANSFORMATION = [[1,270], [-1,180],[1,180]] # right arm
ANGLE_LIMITS = [[-100,10], [-10,100], [-10,100]]
RUN_MOTORS = 0
STEP_DIFF = 50

human_angles = open("human_angles.csv","w")
manipulator_angles = open("manipulator_angles.csv","w")

delay_data = []

gripper_state = 0
prev_steps = []
at_init_pos = False
once = True
if RUN_MOTORS:
    my_arm_controller = arm_control_utils.arm_controller(DXL_ID,devicename='/dev/ttyUSB0', baudrate=1000000)
    my_arm_controller.initialize_motors()
    my_arm_controller.enable_state_torque()
    for id in UNUSED_ID:
        my_arm_controller.enable_torque(id)
    my_arm_controller.enable_torque(GRIPPER_ID)
    gripper_move(0)

client = paho.Client("client_001", clean_session=True)
client.on_connect = on_connect
client.on_message = on_message
print("Connecting to broker: ", broker)
client.max_queued_messages_set(256)
client.connect(broker, 1883)
client.loop_start()
q = queue.Queue()



while True:
    message = q.get(timeout=1000)
    msg = json.loads(message)
    angles =[msg['shoulder']['pitch'], msg['shoulder']['yaw'], msg['elbow']['pitch']]
    gripper = msg['is_gripper_close']
    if once:
        init_msg_timestamp = msg['timestamp']
        start_time = time.time()
        once = False
    t = time.time() - start_time
    print(t)
    print(msg['timestamp'])
    delay_data.append(t)
    time.sleep(0.05)
    

    out_of_limits = False
    for i in range(len(angles)):
        # print(ANGLE_LIMITS[i][0])
        if (angles[i] < ANGLE_LIMITS[i][0] or angles[i] > ANGLE_LIMITS[i][1]):
            print("Out of limits. ID", DXL_ID[i])
            out_of_limits = True
            break
    if out_of_limits:
        continue
        
    steps = angle_to_step(angles, TRANSFORMATION)
    
    # print(steps)
    if not at_init_pos:
        if RUN_MOTORS:
            go_to_start_pos(steps)
        at_init_pos = True
        q.empty()
        prev_steps = steps
        

    else:
        # print(steps)
        # print(prev_steps)
        

        n_max = -1
        for i in range(JOINTS):
            n_max = max(n_max, abs(int((steps[i] - prev_steps[i])/STEP_DIFF)))

        # print(prev_steps)
        # print(n_max)
        
        
        split_angles = np.linspace(prev_steps, steps, n_max + 2).astype(int)
        # why +2 -> 1024, 1032 and step is 5 (1032-1024)/5 = 1
        # 1 + 2 = 3
        # np.linspace(1024,1032, 3 ) = [1024., 1028., 1032.] is what we need
        # print(len(split_angles))
        # for i in range(len(split_angles) - 1):
        #     if RUN_MOTORS:
        #         my_arm_controller.write_state(split_angles[i+1])
        #         time.sleep(0.05)
        #     print(split_angles[i+1])

        
        # if diff between steps in more than n
        #  konse angle ka stepdiff sabse jyada he
        # uska stepdiff / 32 -> n
        # n parts me sab angles ko tod
        # print()
    
    
    # if gripper != gripper_state:
    #     if RUN_MOTORS: gripper_move(gripper)
    #     gripper_state = gripper_state ^ 1
    #     print("gripper moved")

    prev_steps = steps