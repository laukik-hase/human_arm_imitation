import time
import pypot.dynamixel

SPLINE = 5000
DELAY = 1


def setTraj1(id, duration,coeffs) :
    errorCounter = 0
    delay = 0.001
    while True :
        try:
            dxl_io.set_traj1_size({id:5})
            time.sleep(delay)
            dxl_io.set_duration1({id:duration})
            time.sleep(delay)
            dxl_io.set_a0_traj1({id : coeffs[0]})
            time.sleep(delay)
            dxl_io.set_a1_traj1({id : coeffs[1]})
            time.sleep(delay)
            dxl_io.set_a2_traj1({id : coeffs[2]})
            time.sleep(delay)
            dxl_io.set_a3_traj1({id : coeffs[3]})
            time.sleep(delay)
            dxl_io.set_a4_traj1({id : coeffs[4]})
            time.sleep(delay)
            break
        except:
            errorCounter = errorCounter + 1
            # print "Nope :/"
            break
#         print "Nb errors : ", errorCounter

def setTorque1(id, duration,coeffs) :
    errorCounter = 0
    delay = 0.001
    while True :
        try:
            dxl_io.set_torque1_size({id:5})
            time.sleep(delay)
            dxl_io.set_duration1({id:duration})
            time.sleep(delay)
            dxl_io.set_a0_torque1({id : coeffs[0]})
            time.sleep(delay)
            dxl_io.set_a1_torque1({id : coeffs[1]})
            time.sleep(delay)
            dxl_io.set_a2_torque1({id : coeffs[2]})
            time.sleep(delay)
            dxl_io.set_a3_torque1({id : coeffs[3]})
            time.sleep(delay)
            dxl_io.set_a4_torque1({id : coeffs[4]})
            time.sleep(delay)
            break
        except:
            errorCounter = errorCounter + 1
            # print "Nope :/"
            pass
#         print "Nb errors : ", errorCounter


def setTraj2(id, duration, coeffs) :
    errorCounter = 0
    delay = 0.001

    while True :
        try:
            dxl_io.set_traj2_size({id:5})
            time.sleep(delay)
            dxl_io.set_duration2({id:duration})
            time.sleep(delay)
            dxl_io.set_a0_traj2({id : coeffs[0]})
            time.sleep(delay)
            dxl_io.set_a1_traj2({id : coeffs[1]})
            time.sleep(delay)
            dxl_io.set_a2_traj2({id : coeffs[2]})
            time.sleep(delay)
            dxl_io.set_a3_traj2({id : coeffs[3]})
            time.sleep(delay)
            dxl_io.set_a4_traj2({id : coeffs[4]})
            time.sleep(delay)
            break
        except:
            errorCounter = errorCounter + 1
#                 print "nb errors = ", errorCounter
            break
#         print "Nb errors : ", errorCounter

def setTorque2(id, duration, coeffs) :
    errorCounter = 0
    delay = 0.001
    while True :
        try:
            dxl_io.set_torque2_size({id:5})
            time.sleep(delay)
            dxl_io.set_duration2({id:duration})
            time.sleep(delay)
            dxl_io.set_a0_torque2({id : coeffs[0]})
            time.sleep(delay)
            dxl_io.set_a1_torque2({id : coeffs[1]})
            time.sleep(delay)
            dxl_io.set_a2_torque2({id : coeffs[2]})
            time.sleep(delay)
            dxl_io.set_a3_torque2({id : coeffs[3]})
            time.sleep(delay)
            dxl_io.set_a4_torque2({id : coeffs[4]})
            time.sleep(delay)
            break
        except:
            errorCounter = errorCounter + 1
            # print "Nope :/"
            pass
#         print "Nb errors : ", errorCounter
        
def singleMotorTest(dxl_io) :
        print("Test with PID only:")
        dxl_io.set_mode_dynaban({4:0})
        time.sleep(0.1)
        dxl_io.enable_torque({4:1})
        time.sleep(0.1)
        dxl_io.set_goal_position({4:90})
        time.sleep(1)
        dxl_io.set_pid_gain({4:[2,0,0]})
        time.sleep(0.1)

        print("Setting traj1 :")
        setTraj1(4, SPLINE, [0.0 - 1024.0, 532.49287882689293, 29.07849099701108, -1058.1470413492355, 459.3664329672057])

        print("Setting mode and tracking :")
        dxl_io.set_mode_dynaban({1:2}) # 1 = predictive only # 2 = PID only, # 3 = PID + predictive, # 4 = compliant kind of, # 5 = no strings attached
        print("Sleeping")
        time.sleep(DELAY)
        while True :
            print("Traj2")
            setTraj2(4, SPLINE, [169.9973127875532 - 1024.0, 1.2118904739507608, -859.49525560910968, 109.93882674890278, 489.17556618589202])
            dxl_io.set_copy_next_buffer({4:1}) # buffer 2 will be copied into buffer 1 once the traj 1 finishes
            time.sleep(DELAY)
            print("Traj3")
            setTraj2(4, SPLINE, [0.0 - 1024.0, -532.49287882689202, -29.078490997017791, 1058.1470413492527, -459.36643296722087])
            dxl_io.set_copy_next_buffer({4:1}) # buffer 2 will be copied into buffer 1 once the traj 1 finishes
            time.sleep(DELAY)
            print("Traj4")
            setTraj2(4, SPLINE, [-169.99731278755326 - 1024.0, -1.2118904739506096, 859.49525560910888, -109.93882674889758, -489.17556618590021])
            dxl_io.set_copy_next_buffer({4:1}) # buffer 2 will be copied into buffer 1 once the traj 1 finishes
            time.sleep(DELAY)
            
            print("Traj1")
            setTraj2(4, SPLINE, [0.0 - 1024.0, 532.49287882689293, 29.07849099701108, -1058.1470413492355, 459.3664329672057])
            dxl_io.set_copy_next_buffer({4:1}) # buffer 2 will be copied into buffer 1 once the traj 1 finishes
            time.sleep(DELAY)
            
        print("End")
        dxl_io.set_mode_dynaban({1:0})
        # dxl_io.set_goal_position({1:0})
        time.sleep(1)


dxl_io = pypot.dynamixel.DxlIO('/dev/ttyUSB0', baudrate=1000000)
singleMotorTest(dxl_io)