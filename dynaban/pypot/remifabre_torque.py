import time
import numpy
import json

# pypot imports
import pypot.dynamixel

num = 3

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
            dxl_io.set_traj1_size({id: 3})
            time.sleep(delay)
            dxl_io.set_duration1({id: duration})
            time.sleep(delay)
            dxl_io.set_a0_traj1({id: coeffs[0]})
            time.sleep(delay)
            dxl_io.set_a1_traj1({id: coeffs[1]})
            time.sleep(delay)
            dxl_io.set_a2_traj1({id: coeffs[2]})
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
            dxl_io.set_traj2_size({id: 3})
            time.sleep(delay)
            dxl_io.set_duration2({id: duration})
            time.sleep(delay)
            dxl_io.set_a0_traj2({id: coeffs[0]})
            time.sleep(delay)
            dxl_io.set_a1_traj2({id: coeffs[1]})
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

dxl_io.enable_torque([1])
dxl_io.enable_torque([2])
dxl_io.enable_torque([3])
dxl_io.enable_torque([4])
print ("Test with PID only:")
dxl_io.set_mode_dynaban({num:0})
time.sleep(0.1)
dxl_io.enable_torque({num:1})
time.sleep(0.1)
# dxl_io.set_goal_position({4:90})
# time.sleep(1)
# dxl_io.set_pid_gain({4:[1,0,0]})
# time.sleep(0.1)

setTraj1(num, 40000, [2048.0, 0, 0.0])
# setTorque1(num, 40000, [0.0,0.0,0.0])        
print ("Setting mode and tracking :")

dxl_io.set_mode_dynaban({num:2}) 
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