import paho.mqtt.client as paho 
import time
import Queue as queue
import json
import real_time_manipulator_math_utils
import pprint
import dynaban as dnb
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

SPLINE = 1
WINDOWSIZE = 5
# MOTOR_ID = [1,2,3,4]
MOTOR_ID = [1]
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

while loop_flag==1:
#     try: 
        message = q.get()
        msg = json.loads(message)
        print(msg['timestamp'])
        if mqtt_initval_error:
            mqtt_initval_error = False
            continue

    #     if msg['is_gripper_close'] !=  gripper_state:
    #         gripper_state = gripper_state^True
    #         if gripper_state is True:
    #             db.close_gripper()
    #         else:
    #             db.open_gripper() 

        if first_val:
            init_timestamp = msg['timestamp']
            init_angle = [-msg['shoulder']['pitch'],msg['shoulder']['roll'],msg['shoulder']['yaw'],msg['elbow']['pitch']]
#             db.init_motor()
#             db.go_to_start_pos(init_angle)
            first_val = False
        else:
            # collect till SPLINE
            if(msg['timestamp'] - init_timestamp > SPLINE):

                init_timestamp = msg['timestamp']
                
                for j in range(JOINTS):
                    if padded_angles == []:
                        angles[j] = [angles[j][0]]*(WINDOWSIZE-1) + angles[j]
                    else:
                        angles[j] = padded_angles[j] + angles[j]

                padded_angles = [ angles[j][-(WINDOWSIZE-1):] for j in range(JOINTS) ]


                angles = math_utils_obj.real_time_moving_average(angles,WINDOWSIZE)
    #             transformation = [[-1,270],[-1,270],[1,90],[1,90]]
                transformation = [[-1,180]]
                angles = math_utils_obj.angles_to_steps(angles, transformation)
                coeff_angle = math_utils_obj.calculate_coefficients_angles(timestamps, angles)
                print("angles coefficients")
                pp.pprint(coeff_angle)

                if once:
                    for joints in range(len(MOTOR_ID)):
                        db.setTraj1(MOTOR_ID[joints],SPLINE*10000, [coeff_angle[joints][3],coeff_angle[joints][2],coeff_angle[joints][1],coeff_angle[joints][0]])

#                         self.setTorque1(self.MOTOR_ID[joints],spline*10000, [coeff_torque[joints][3],coeff_torque[joints][2],coeff_torque[joints][1],coeff_torque[joints][0]])

                    db.dxl_io.set_mode_dynaban(db.DXL_DICT_3)
                    # db.dxl_io.set_mode_dynaban({2:3})
#                     print("set mode")
                    
                    once = False
            

                else:
                    for joints in range(len(MOTOR_ID)):
                        db.setTraj2(MOTOR_ID[joints],SPLINE*10000, [coeff_angle[joints][3],coeff_angle[joints][2],coeff_angle[joints][1],coeff_angle[joints][0]])
                        

#                         self.setTorque2(self.MOTOR_ID[joints],spline*10000, [coeff_torque[joints][3],coeff_torque[joints][2],coeff_torque[joints][1],coeff_torque[joints][0]])
           
#             self.dxl_io.set_copy_next_buffer(self.DXL_DICT_1)
                    
                    
                    db.dxl_io.set_copy_next_buffer(db.DXL_DICT_1)
                    
                    traj1 = []
                    for joints in range(len(MOTOR_ID)):
                        
                        traj1.append([db.dxl_io.get_a0_traj1([MOTOR_ID[joints]])[0],db.dxl_io.get_a1_traj1([MOTOR_ID[joints]])[0],db.dxl_io.get_a2_traj1([MOTOR_ID[joints]])[0],db.dxl_io.get_a3_traj1([MOTOR_ID[joints]])[0]])
                    
                    if last_flag:
                        prev_traj2 = traj1
                        last_flag = False
#                     print(prev_traj2[0][0])  
#                     db.dxl_io.set_a0_traj1({1: 2306.0})
                    if prev_traj2 != traj1:
                        for joints in range(len(MOTOR_ID)):
                            db.dxl_io.set_a0_traj1({MOTOR_ID[joints] : prev_traj2[joints][0]})
                            db.dxl_io.set_a1_traj1({MOTOR_ID[joints] : prev_traj2[joints][1]})
                            db.dxl_io.set_a2_traj1({MOTOR_ID[joints] : prev_traj2[joints][2]})
                            db.dxl_io.set_a3_traj1({MOTOR_ID[joints] : prev_traj2[joints][3]})
                    
                    
                    traj2 = []
                    
                    for joints in range(len(MOTOR_ID)):
                         prev_traj2.append([db.dxl_io.get_a0_traj2([MOTOR_ID[joints]])[0],db.dxl_io.get_a1_traj2([MOTOR_ID[joints]])[0],db.dxl_io.get_a2_traj2([MOTOR_ID[joints]])[0],db.dxl_io.get_a3_traj2([MOTOR_ID[joints]])[0]])
    
                               
                    print("get a1 for traj1" ,db.dxl_io.get_a0_traj1([MOTOR_ID[0]]))
                    print("get a1 for traj2" ,db.dxl_io.get_a0_traj2([MOTOR_ID[0]]))
#                 time.sleep(1)

                    print("set buffer")


                # empty
                timestamps = []
                angles = []
                [angles.append([]) for j in range(JOINTS)]
                torques = []
                [torques.append([]) for j in range(JOINTS)]

        timestamps.append(msg['timestamp'] - init_timestamp)
        angles[0].append(msg['shoulder']['pitch'])
    #     angles[1].append(msg['shoulder']['pitch'])
    #     angles[2].append(msg['shoulder']['pitch'])
    #     angles[3].append(msg['shoulder']/['pitch'])
    #     angles[1].append(msg['shoulder']['roll'])
    #     angles[2].append(msg['shoulder']['yaw'])
    #     angles[3].append(msg['elbow']['pitch'])
#     except:
#         print("termios error")
#         pass

client.disconnect()
client.loop_stop()