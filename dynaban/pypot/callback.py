import paho.mqtt.client as paho 
import time
import Queue as queue
import json
import real_time_manipulator_math_utils
import pprint
pp = pprint.PrettyPrinter(indent=4)
# import rbdl
# import manip motion





def on_connect(client, userdata, flags, rc):
    client.subscribe(topic, qos)

def on_message(client, userdata, message):
    msg = message.payload.decode("utf-8")
    q.put(msg)
    # print("Received: ", msg)

broker = "test.mosquitto.org"
topic = "fyp/sensors"
qos = 0

client = paho.Client("client_001")     
client.on_connect=on_connect
client.on_message = on_message        
client.connect(broker)      
client.loop_start()
loop_flag=1
q = queue.Queue()


JOINTS = 4
SPLINE = 1
WINDOWSIZE = 5

math_utils_obj = real_time_manipulator_math_utils.manipulator_math_utils(JOINTS)
timestamps = []
angles = []
[angles.append([]) for j in range(JOINTS)]
torques = []
[torques.append([]) for j in range(JOINTS)]
padded_angles = []
first_val = True
laukik_tatti = True

while loop_flag==1:
    message = q.get()
    msg = json.loads(message)
    if laukik_tatti:
        laukik_tatti = False
        continue


    if first_val:
        init_timestamp = msg['timestamp']
        first_val = False
    else:
        # collect till SPLINE
        if(msg['timestamp'] - init_timestamp > SPLINE):
            # print(timestamps)
            
            init_timestamp = msg['timestamp']
            
            # padding for smooth transition in moving average
            for j in range(JOINTS):
                if padded_angles == []:
                    angles[j] = [angles[j][0]]*(WINDOWSIZE-1) + angles[j]
                else:
                    angles[j] = padded_angles[j] + angles[j]
            
            
            
            # moving average with length similar to timestamps
            print("raw angles")
            pp.pprint(angles)
            padded_angles = [ angles[j][-(WINDOWSIZE-1):] for j in range(JOINTS) ]


            angles = math_utils_obj.real_time_moving_average(angles)
            print("angles after moving avg")
            pp.pprint(angles)
            # torques = get torque from rbdl (timestamp, angles)

            # convert angles to steps
            transformation = [[1,0]]*JOINTS
            angles = math_utils_obj.angles_to_steps(angles, transformation)
            print("angles to steps")
            pp.pprint(angles)

            # call to get coeffs
            angle_coeffs = math_utils_obj.calculate_coefficients_angles(timestamps, angles)
            print("angles coefficients")
            pp.pprint(angle_coeffs)
            # set motion on manipulator

            
            # empty
            timestamps = []
            angles = []
            [angles.append([]) for j in range(JOINTS)]
            torques = []
            [torques.append([]) for j in range(JOINTS)]

    timestamps.append(msg['timestamp'] - init_timestamp)
    angles[0].append(msg['shoulder']['pitch'])
    angles[1].append(msg['shoulder']['roll'])
    angles[2].append(msg['shoulder']['yaw'])
    angles[3].append(msg['elbow']['pitch'])

client.disconnect()
client.loop_stop()