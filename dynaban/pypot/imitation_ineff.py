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
# import rbdl
# import manip motion


def on_connect(client, userdata, flags, rc):
    client.subscribe(topic, qos)

def on_message(client, userdata, message):
    msg = message.payload.decode("utf-8")
    q.put(msg)
#     print("Received: ", msg)

broker = "broker.hivemq.com"
topic = "fyp/sensors"
qos = 0

client = paho.Client("client_001")     
client.on_connect=on_connect
client.on_message = on_message        
client.connect(broker)      
client.loop_start()
loop_flag=1
q = queue.Queue()

SPLINE = 1
WINDOWSIZE = 5
# MOTOR_ID = [1,2,3,4]
MOTOR_ID = [1,2]
angle_limit = [[150,350],[90,270]]
JOINTS = len(MOTOR_ID)
GRIPPER_ID = 5

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
traj1 = []
prev_traj2 = []
once = True
last_flag = True
coeff_angle = []

ports = pypot.dynamixel.get_available_ports()

if not ports:
    raise IOError('no port found!')

print('ports found', ports)

print('connecting on the first available port:', ports[0])

dxl_io = pypot.dynamixel.DxlIO(ports[0])
#         dxl_io.enable_torque({self.GRIPPER_ID: 1})
time.sleep(0.1)    


def init_motor():

    for joints in range(len(MOTOR_ID)):

        num = MOTOR_ID[joints]
        dxl_io.set_mode_dynaban({num:1})
#             time.sleep(0.1)
        dxl_io.enable_torque({num:1})
#             time.sleep(0.1)
#         dxl_io.set_pid_gain({MOTOR_ID[joints]:[1,0,0]})
#             time.sleep(0.1)
    dxl_io.enable_torque({1:1,2:1,3:1,4:1})
    dxl_io.set_angle_limit({1:[-10.0,150.0],2:[-90.0,90.0]})

def reset_motor():
    
    for joints in range(len(MOTOR_ID)):

        num = MOTOR_ID[joints]
        dxl_io.set_mode_dynaban({num:0})
        dxl_io.enable_torque({num:0})
    time.sleep(0.1)
    init_motor()
    
def clear_reg():

    for joints in range(len(MOTOR_ID)):
        
        dxl_io.set_a0_traj1({MOTOR_ID[joints] : 0.0})
        dxl_io.set_a1_traj1({MOTOR_ID[joints] : 0.0})
        dxl_io.set_a2_traj1({MOTOR_ID[joints] : 0.0})
        dxl_io.set_a3_traj1({MOTOR_ID[joints] : 0.0})
                    

def rewrite_reg(traj2):
    
    for joints in range(len(MOTOR_ID)):
        
        dxl_io.set_a0_traj1({MOTOR_ID[joints] : traj2[joints][0]})
        dxl_io.set_a1_traj1({MOTOR_ID[joints] : traj2[joints][1]})
        dxl_io.set_a2_traj1({MOTOR_ID[joints] : traj2[joints][2]})
        dxl_io.set_a3_traj1({MOTOR_ID[joints] : traj2[joints][3]})
#                     traj2 = []
    
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
                    time.sleep(0.02) 
                else:
                    dxl_io.set_goal_position({MOTOR_ID[joints]:current_pos + i})
                    time.sleep(0.02)
        q.empty()

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
#             dxl_io.set_a4_traj1({id: coeffs[4]})
#             time.sleep(delay)

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
#             dxl_io.set_a4_traj2({id: coeffs[4]})
#             time.sleep(delay)

            break
        except:
            errorCounter = errorCounter + 1
            print("nb errors2 = ", errorCounter)
            break

                
while loop_flag==1:
        # threading.enumerate()
#     try: 
        message = q.get()
        msg = json.loads(message)
#         print(msg['timestamp'])
        if mqtt_initval_error:
            mqtt_initval_error = False
            continue

        if first_val:
            init_timestamp = msg['timestamp']
#             init_angle = [-msg['shoulder']['pitch'],msg['shoulder']['roll'],msg['shoulder']['yaw'],-msg['elbow']['pitch']-90]
#             init_angle = [-msg['shoulder']['pitch'],-msg['elbow']['pitch']-90]
            init_angle = [-msg['shoulder']['pitch'],msg['shoulder']['roll']]
            init_motor()
            go_to_start_pos(init_angle)
            first_val = False
            
#         current_angle = []    
#         for joints in range(len(MOTOR_ID)):
#             current_angle.append(dxl_io.get_present_position([MOTOR_ID[joints]])[0])
            
#         print("########",current_angle)
#         for joints in range(len(MOTOR_ID)):
#             if ((abs(current_angle[joints]) > angle_limit[joints][0] and abs(current_angle[joints]) < angle_limit[joints][1])):
#                 dxl_io.enable_torque({1:0,2:0,3:0,4:0})                  
#                 sys.exit()
        else:

            if(msg['timestamp'] - init_timestamp >= SPLINE):

                init_timestamp = msg['timestamp']
                
                for j in range(JOINTS):
                    if padded_angles == []:
                        angles[j] = [angles[j][0]]*(WINDOWSIZE-1) + angles[j]
                    else:
                        angles[j] = padded_angles[j] + angles[j]

                padded_angles = [ angles[j][-(WINDOWSIZE-1):] for j in range(JOINTS) ]


                angles = math_utils_obj.real_time_moving_average(angles,WINDOWSIZE)
    #             transformation = [[-1,270],[-1,270],[1,90],[1,90]]
#                 transformation = [[-1,180],[1,180],[-1,180],[-1,90]]
                transformation = [[-1,180],[1,180]]
                angles = math_utils_obj.angles_to_steps(angles, transformation)
                coeff_angle = math_utils_obj.calculate_coefficients_angles(timestamps, angles)
#                 print("angles coefficients")
#                 pp.pprint(coeff_angle)


            if coeff_angle != []:
                if once:
                    for joints in range(len(MOTOR_ID)):
                        setTraj1(MOTOR_ID[joints],SPLINE*10000, [coeff_angle[joints][3],coeff_angle[joints][2],coeff_angle[joints][1],coeff_angle[joints][0]])

                    dxl_io.set_mode_dynaban(db.DXL_DICT_3)
#                     dxl_io.set_mode_dynaban({1:3})            

                    coeff_angle = []
                    once = False
            

                else:
#                     print(coeff_angle)
                    for joints in range(len(MOTOR_ID)):
                        setTraj2(MOTOR_ID[joints],SPLINE*10000, [coeff_angle[joints][3],coeff_angle[joints][2],coeff_angle[joints][1],coeff_angle[joints][0]])
                    
                    dxl_io.set_copy_next_buffer(db.DXL_DICT_1)

#                     dxl_io.set_copy_next_buffer({2:1})
                    

                            
                    
                                              
                    traj1 = []
                    for joints in range(len(MOTOR_ID)):
                        
                        traj1.append([dxl_io.get_a0_traj1([MOTOR_ID[joints]])[0],dxl_io.get_a1_traj1([MOTOR_ID[joints]])[0],dxl_io.get_a2_traj1([MOTOR_ID[joints]])[0],dxl_io.get_a3_traj1([MOTOR_ID[joints]])[0]])
                    
                    traj2 = []
                    for joints in range(len(MOTOR_ID)):
                         traj2.append([dxl_io.get_a0_traj2([MOTOR_ID[joints]])[0],dxl_io.get_a1_traj2([MOTOR_ID[joints]])[0],dxl_io.get_a2_traj2([MOTOR_ID[joints]])[0],dxl_io.get_a3_traj2([MOTOR_ID[joints]])[0]])
                            
                    if last_flag:
                        prev_traj2 = traj1
                        last_flag = False
#                     print(prev_traj2[0][0])  
#                     dxl_io.set_a0_traj1({1: 2306.0})
                        
                    
#                     print("traj1" ,traj1)
#                     print("prev_traj2" , prev_traj2)
            
                    if prev_traj2 != traj1:
                        
                        while True:
#                             print((not dxl_io.is_moving([MOTOR_ID[0]])[0]) and (not dxl_io.is_moving([MOTOR_ID[1]])[0]) and (not dxl_io.is_moving([MOTOR_ID[2]])[0]) and (not dxl_io.is_moving([MOTOR_ID[3]])[0]))
                            
#                             if ((not dxl_io.is_moving([MOTOR_ID[0]])[0]) and (not dxl_io.is_moving([MOTOR_ID[1]])[0]) and (not dxl_io.is_moving([MOTOR_ID[2]])[0]) and (not dxl_io.is_moving([MOTOR_ID[3]])[0])):
                            if (not dxl_io.is_moving([MOTOR_ID[0]])[0] ): 
                            
#                                 print("#############################")
#                                 print("not same")
#                                 reset_motor()
#                                 clear_reg()
                                rewrite_reg(prev_traj2)
                                dxl_io.set_mode_dynaban(db.DXL_DICT_3)
                                break

                            
                            
#                         dxl_io.set_mode_dynaban({1:3})
                        
#                         
                    prev_traj2 = []
                    for joints in range(len(MOTOR_ID)):
                         prev_traj2.append([dxl_io.get_a0_traj2([MOTOR_ID[joints]])[0],dxl_io.get_a1_traj2([MOTOR_ID[joints]])[0],dxl_io.get_a2_traj2([MOTOR_ID[joints]])[0],dxl_io.get_a3_traj2([MOTOR_ID[joints]])[0]])

    #                         self.setTorque2(self.MOTOR_ID[joints],spline*10000, [coeff_torque[joints][3],coeff_torque[joints][2],coeff_torque[joints][1],coeff_torque[joints][0]])
            
                    
                
#                     dxl_io.set_copy_next_buffer(db.DXL_DICT_1)
                   
                    coeff_angle = []
                    time.sleep(SPLINE)
#                     print("get a1 for traj1" ,dxl_io.get_a0_traj1([MOTOR_ID[0]]))
#                     print("get a1 for traj2" ,dxl_io.get_a0_traj2([MOTOR_ID[0]]))



                # empty
                timestamps = []
                angles = []
                [angles.append([]) for j in range(JOINTS)]
                torques = []
                [torques.append([]) for j in range(JOINTS)]

        timestamps.append(msg['timestamp'] - init_timestamp)
        angles[0].append(msg['shoulder']['pitch'])
        angles[1].append(msg['shoulder']['roll'])
#         angles[0].append(msg['shoulder']['pitch'])
    #     angles[3].append(msg['shoulder']/['pitch'])
#         angles[1].append(msg['shoulder']['roll'])
#         angles[2].append(msg['shoulder']['yaw'])
#         angles[3].append(msg['elbow']['pitch'])
#     except:
#         print("termios error")
#         pass

client.disconnect()
client.loop_stop()