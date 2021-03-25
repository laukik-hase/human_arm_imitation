import time
import numpy
import json

# pypot imports
import pypot.dynamixel


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


print("Test with PID only:")
time.sleep(0.1)
# dxl_io.enable_torque({4: 1})
time.sleep(0.1)
# 1 = predictive only # 2 = PID only, # 3 = PID + predictive, # 4 = compliant kind of, # 5 = no strings attached


# l = 0.085
# #Torque = m*g*l
# g = 9.81
# m = 0.2
# torque = m*g*l*4096/numpy.pi

# dxl_io.set_angle_limit({4: (3100,4096)})

dxl_io.set_mode_dynaban({4: 0})
dxl_io.enable_torque({4: 0})

print("Setting traj1 :")
setTraj1(4, 20000, [3072.0, -512.0, 0.0])

# print("Setting torque ...")
# setTorque1(4, 20000, [1023.0,0.0,0.0])

print("Setting mode and tracking :")

dxl_io.enable_torque({4: 1})
dxl_io.set_mode_dynaban({4: 2})
# time.sleep(0.25)
# setTraj2(4, 30000, [2048, 512.0, 0.0])
# dxl_io.set_copy_next_buffer({4: 1})
# time.sleep(3)
# dxl_io.set_mode_dynaban({4: 0})
# dxl_io.enable_torque({4: 0})
# setTraj1(4, 20000, [2048.0, 512.0, 0.0])

# # print("Setting torque ...")
# # setTorque1(4, 20000, [1023.0,0.0,0.0])

# print("Setting mode and tracking :")

# dxl_io.enable_torque({4: 1})
# dxl_io.set_mode_dynaban({4: 2})

# time.sleep(0.5)
# dxl_io.set_mode_dynaban({4: 0})
# dxl_io.enable_torque({4: 0})


# time.sleep(0.5)
# setTraj2(4, 20000, [3072.0, -512.0, 0.0])
# dxl_io.set_copy_next_buffer({4: 1})
# time.sleep(0.5)

# dxl_io.set_mode_dynaban({4: 0})
# # dxl_io.set_goal_position({1:0})
# time.sleep(1)

# dxl_io.set_mode_dynaban({4: 0})
# dxl_io.enable_torque({4: 0})

# print("Setting traj1 :")
# setTraj1(4, 20000, [2048.0])

# print("Setting torque ...")
# setTorque1(4, 20000, [1023.0])

# print("Setting mode and tracking :")
# # 1 = predictive only # 2 = PID only, # 3 = PID + predictive, # 4 = compliant kind of, # 5 = no strings attached
# dxl_io.enable_torque({4: 1})
# dxl_io.set_mode_dynaban({4: 3})

# print("Sleeping")
# time.sleep(2)
