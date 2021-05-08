import paho.mqtt.client as paho 
import time
import Queue as queue
import json
import real_time_manipulator_math_utils
import pprint
import dynaban as dnb
import time
import pypot.robot
import pypot.dynamixel
import snoop 
import threading
import sys

pp = pprint.PrettyPrinter(indent=4)




SPLINE = 1
WINDOWSIZE = 5
# MOTOR_ID = [1,2,3,4]
MOTOR_ID = [1]
JOINTS = len(MOTOR_ID)
GRIPPER_ID = 5



 

def init_motor():

    for joints in range(len(MOTOR_ID)):

        num = MOTOR_ID[joints]
        dxl_io.set_mode_dynaban({1:0})
        dxl_io.enable_torque({num:1})


def rewrite_reg(id, traj2):
        
    dxl_io.set_a0_traj1({MOTOR_ID[id] : traj2[id][0]})
    dxl_io.set_a1_traj1({MOTOR_ID[id] : traj2[id][1]})
    dxl_io.set_a2_traj1({MOTOR_ID[id] : traj2[id][2]})
    dxl_io.set_a3_traj1({MOTOR_ID[id] : traj2[id][3]})
    
def go_to_start_pos(init_angle):
        
    for joints in range(len(MOTOR_ID)):
        num = MOTOR_ID[joints]
        start_angle = int(init_angle[joints])
        print("start angle",start_angle)
        current_pos = int(dxl_io.get_present_position([num])[0])
        diff = abs(current_pos-start_angle)
        print("diff", diff)
        for i in range(diff):
            if (current_pos > start_angle):
                dxl_io.set_goal_position({MOTOR_ID[joints]:current_pos - i})
                time.sleep(0.05) 
            else:
                dxl_io.set_goal_position({MOTOR_ID[joints]:current_pos + i})
                time.sleep(0.05)

def setTraj1( id, duration, coeffs):
    errorCounter = 0
    delay = 0.001
    while True:
        try:
            dxl_io.set_traj1_size({id: 4})
            time.sleep(delay)
            dxl_io.set_duration1({id: duration})
            time.sleep(delay)
            dxl_io.set_a0_traj1({id: coeffs[0]})
            time.sleep(delay)
            dxl_io.set_a1_traj1({id: coeffs[1]})
            time.sleep(delay)
            dxl_io.set_a2_traj1({id: coeffs[2]})
            time.sleep(delay)
            dxl_io.set_a3_traj1({id: coeffs[3]})
            time.sleep(delay)
            break
        except:
            errorCounter = errorCounter + 1
            # print "Nope :/"
            break
            print("nb errors1 = ", errorCounter)


def setTraj2(id, duration, coeffs):
    errorCounter = 0
    delay = 0.001

    while True:
        try:
            dxl_io.set_traj2_size({id: 4})
            time.sleep(delay)
            dxl_io.set_duration2({id: duration})
            time.sleep(delay)
            dxl_io.set_a0_traj2({id: coeffs[0]})
            time.sleep(delay)
            dxl_io.set_a1_traj2({id: coeffs[1]})
            time.sleep(delay)
            dxl_io.set_a2_traj2({id: coeffs[2]})
            time.sleep(delay)
            dxl_io.set_a3_traj2({id: coeffs[3]})
            time.sleep(delay)
            break
        except:
            errorCounter = errorCounter + 1
            print("nb errors2 = ", errorCounter)
            break

def on_connect(client, userdata, flags, rc):
    client.subscribe(topic, qos)


def on_message(client, userdata, message):
    msg = message.payload.decode("utf-8")
    q.put(msg)

# Class Thread, taking the thread name and the thread function
class Thread (threading.Thread):
    def __init__(self, threadID, name, func):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.func = func  # The thread function is passed as a pointer

    def run(self):
        self.func(self.name)


def manip_motion(threadName):
        # TODO: Capture values for 1 sec interval; as soon as this is done, acquire a lock for the manipulator motion
        # Only after completion of motion, the lock shall be set free.
    # preprocessing_time = time.time()               
    loop_flag = 1
    count = 1
    db = dnb.dynaban(MOTOR_ID, GRIPPER_ID)
    math_utils_obj = real_time_manipulator_math_utils.manipulator_math_utils(JOINTS)
    timestamps = []
    angles = []
    [angles.append([]) for j in range(JOINTS)]
    torques = []
    [torques.append([]) for j in range(JOINTS)]
    padded_angles = []
    first_val = True
    mqtt_initval_error = True
    gripper_state = False # false = open
    once = True
    cats_transform = [[1,180],[1,180],[-1,180],[1,180]] #current an
    start_time = time.time()
    while loop_flag==1:
        if exitFlag:
            threadName.exit()
        try:
            time.sleep(0.05)
            message = q.get(block=False)
            msg = json.loads(message)

        
            
            # print("q.get ", time.time() - preprocessing_time)

            if count == 1:  # go to start position
                init_timestamp = msg['timestamp']
                init_angle = [-msg['shoulder']['pitch']]

                init_motor()
                print("init angle", init_angle)
                go_to_start_pos(init_angle)
                count = 2
            
            else:
                if(msg['timestamp'] - init_timestamp >= SPLINE): # if one second is complete
                    init_timestamp = msg['timestamp']
                    # print("1 second hua", time.time() - preprocessing_time)
                    
                    for j in range(JOINTS):
                        if padded_angles == []:
                            angles[j] = [angles[j][0]]*(WINDOWSIZE-1) + angles[j]
                        else:
                            angles[j] = padded_angles[j] + angles[j]

                    padded_angles = [ angles[j][-(WINDOWSIZE-1):] for j in range(JOINTS) ]

                    angles = math_utils_obj.real_time_moving_average(angles,WINDOWSIZE)
                    transformation = [[-1,180]]
                    angles = math_utils_obj.angles_to_steps(angles, transformation)
                    coeff_angle = math_utils_obj.calculate_coefficients_angles(timestamps, angles)
                    print(coeff_angle)
                    print("preprocessing ", time.time() - start_time)
                    start_time = time.time()
                    # if once: 
                    #     for joints in range(len(MOTOR_ID)):
                    #         setTraj1(MOTOR_ID[joints],SPLINE*10000, [coeff_angle[joints][3],coeff_angle[joints][2],coeff_angle[joints][1],coeff_angle[joints][0]])
                    #     print(coeff_angle)
                    #     dxl_io.set_mode_dynaban(db.DXL_DICT_3)
                    #     once = False

                    # else:
                    #     for i in range(len(MOTOR_ID)):
                    #         rewrite_reg(i, [[coeff_angle[i][3],coeff_angle[i][2],coeff_angle[i][1],coeff_angle[i][0]]])
                    #         dxl_io.set_mode_dynaban({MOTOR_ID[i]:3})

                    # print("set mode dynaban ", time.time() - preprocessing_time)                

                    timestamps = []
                    angles = []
                    [angles.append([]) for j in range(JOINTS)]
                    torques = []
                    [torques.append([]) for j in range(JOINTS)]
                

            timestamps.append(msg['timestamp'] - init_timestamp)
            angles[0].append(msg['shoulder']['pitch'])

        except:
            print("no val")

ports = pypot.dynamixel.get_available_ports()

if not ports:
    raise IOError('no port found!')

print('ports found', ports)

print('connecting on the first available port:', ports[0])

dxl_io = pypot.dynamixel.DxlIO(ports[0])
time.sleep(0.1)   



exitFlag = 0
broker = "broker.hivemq.com"
topic = "fyp/demo"
qos = 2

start = time.time()
client = paho.Client("client_001")
client.on_connect = on_connect
client.on_message = on_message

q = queue.Queue()
manip_thread = Thread(1, "manip_thread", manip_motion)
manip_thread.start()

client.connect(broker)
client.loop_start()

# exit_flag = 1
# client.loop_stop()
# client.disconnect()