# Code to receive angle values on ros topic /joint_angles
# and move the manipulator based on those angles
# using pypot library

import rospy
import time
from arm_imitation.msg import msg_joint_angles

import numpy
import json

# pypot imports
import pypot.dynamixel

offset = [0, 0, 0, 0]
ID_LIST = [1, 2, 3, 4]
ID_SIZE = len(ID_LIST)

DXL_DICT_1      = dict(zip(ID_LIST, [1]*ID_SIZE))
DXL_DICT_0      = dict(zip(ID_LIST, [0]*ID_SIZE))
DXL_DICT_PID    = dict(zip(ID_LIST, [[1,0,0]]*ID_SIZE))

ports = pypot.dynamixel.get_available_ports()

if not ports:
    raise IOError('no port found!')

print('ports found', ports)

print('connecting on the first available port:', ports[0])
dxl_io = pypot.dynamixel.DxlIO(ports[0])

def setTraj1(id, duration, coeffs):
    errorCounter = 0
    delay = 0.001
    while True:
        try:
            dxl_io.set_traj1_size({id: 5})
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
            dxl_io.set_a4_traj1({id: coeffs[4]})
            time.sleep(delay)
            break
        except:
            errorCounter = errorCounter + 1
            # print "Nope :/"
            break
#         print "Nb errors : ", errorCounter


def setTraj2(id, duration, coeffs):
    errorCounter = 0
    delay = 0.001

    while True:
        try:
            dxl_io.set_traj2_size({id: 5})
            time.sleep(delay)
            dxl_io.set_duration2({id: duration})
            time.sleep(delay)
            dxl_io.set_a0_traj2({id: coeffs[0]})
            time.sleep(delay)
            dxl_io.set_a1_traj2({id: coeffs[1]})
            time.sleep(delay)
            dxl_io.set_a2_traj2({id: coeffs[2]})
            time.sleep(delay)
            dxl_io.set_a2_traj2({id: coeffs[2]})
            time.sleep(delay)
            dxl_io.set_a2_traj2({id: coeffs[2]})
            time.sleep(delay)
            break
        except:
            errorCounter = errorCounter + 1
            print("nb errors = ", errorCounter)
            break


def setTorque1(id, duration, coeffs):
    errorCounter = 0
    delay = 0.001
    while True:
        try:
            dxl_io.set_torque1_size({id: 3})
            time.sleep(delay)
            dxl_io.set_duration1({id: duration})
            time.sleep(delay)
            dxl_io.set_a0_torque1({id: coeffs[0]})
            time.sleep(delay)
            dxl_io.set_a1_torque1({id: coeffs[1]})
            time.sleep(delay)
            dxl_io.set_a2_torque1({id: coeffs[2]})
            time.sleep(delay)
            break
        except:
            errorCounter = errorCounter + 1
            # print "Nope :/"
            pass
#         print "Nb errors : ", errorCounter


# ROS
def callback(data):
    state = [ float(( data.joint_angles[i] + offset[i] ) % 360) for i in range(ID_SIZE) ]
    # write angles to dynamixel
    state = dict(zip(ID_LIST, state))
    print(state)
    dxl_io.set_goal_position(state) # change this to traj1
    # for i in range(ID_SIZE):
    #     dxl_io.set_goal_position({ID_LIST[i]:state[i]})
    time.sleep(0.01)
    
def listener():
    rospy.init_node('arm_controller', anonymous=True)
    rospy.Subscriber("/joint_angles", msg_joint_angles, callback)
    rospy.spin()

dxl_io.set_mode_dynaban(DXL_DICT_1)
time.sleep(0.1)
dxl_io.enable_torque(DXL_DICT_1)
time.sleep(0.1)
dxl_io.set_pid_gain(DXL_DICT_PID)
time.sleep(0.1)

# imitate
print("Ready to imitate")
try:
    listener()
except KeyboardInterrupt:
    # dxl_io.set_mode_dynaban(DXL_DICT_0)
    # time.sleep(0.1)
    dxl_io.enable_torque(DXL_DICT_0)
    time.sleep(0.1)

setTraj1(4, 10000, [2048.0, 0.0, 0.0])        
print ("Setting mode and tracking :")

dxl_io.set_mode_dynaban({4:2}) 
print ("Sleeping")

# while True :
#             print "Sleeping"
#             time.sleep(2.5)
            
#             print "Done. Compliant mode on"
#             dxlIo.enable_torque({1:0})
#             m = float(raw_input("m ?\n"))
#             print "m = ", m, " kg"
#             torque = m*g*l*4096/numpy.pi
#             print "torque = ", torque, " N.m"
#             dxlIo.enable_torque({1:1})
#             print "Setting torque ..."
#             self.setTraj1(1, 20000, [0.0, 0.0, 0.0, 0.0, 0.0])
#             self.setTorque1(1, 20000, [torque, 0.0, 0.0, 0.0, 0.0])
#             self.dxl_io.set_mode_dynaban({1:1})