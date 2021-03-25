
import time
from subprocess import Popen, PIPE
import numpy

class Parameters() :
    def __init__(self, robot, z=0, freq=50, method="constantSpeed", maxAccel=6000, maxSpeed=500) :
#         self.dxlIo = dxlIo
        self.robot = robot
        self.z = z
        self.freq = freq
        self.method = method
        self.maxAccel = maxAccel
        self.maxSpeed = maxSpeed
        self.id1 = 1
        self.id2 = 2
        self.id3 = 3
        self.id4 = 4
        self.ids = [self.id1, self.id2, self.id3, self.id4]

#Doesn't return until finished
def setAnglesToArmSmooth(angles, params, timeToDestination=3) :
    print("Going smoothly to angles = ", angles)
    deltas = []
    initial = []
    i = 0
    for m in params.robot.motors :
        deltas.append(angles[i] - m.present_position)
        initial.append(m.present_position)
        i = i + 1
    
    for i in range(len(deltas)) :
        deltas[i] = deltas[i]%360
        if (deltas[i] > 180) :
            deltas[i] = -360 + deltas[i]
    
    print("Duration = ", timeToDestination, " secs")
    writingPeriod = 1.0/params.freq
    elapsedTime = 0
    while elapsedTime < timeToDestination :
        progress = elapsedTime / timeToDestination
        
        i = 0
        for m in params.robot.motors :
            m.goal_position = initial[i] + progress * deltas[i]
            i = i + 1
            

        elapsedTime = elapsedTime + writingPeriod
        time.sleep(writingPeriod)
        
    print("Done !")

def setAnglesToArm(angles, params) :
    i = 0
    for m in params.robot.motors :
        m.goal_position = angles[i]
        i = i + 1

def setAngleToMotor(id, angle, params, lowLimit=-180, highLimit=180) :
    errorCounter = 0
    if (lowLimit < highLimit) :
        #Inner values
        if (angle < lowLimit) :
            angle = lowLimit
        elif (angle > highLimit):
            angle = highLimit
    else :
        #Outter values (actually works only if lowLimit is positive and highLimit is negative)
        if (angle > 0) :
            if (angle < lowLimit) :
                angle = lowLimit
        else :
            if (angle > highLimit) :
                angle = highLimit

    # print "setting true angle ", angle, " to motor ", id
    while True :
        try:
            params.dxlIo.set_goal_position({id : angle})
            break
        except:
            errorCounter = errorCounter + 1
            # print "Nope :/"
            if errorCounter > 39 :
                print("Nb errors too high : ", errorCounter)
                break
            pass

### Duration in tenth of ms (10000 = 1 sec)
def setTraj1ToMotor(motor, polyCoeffs, duration, copyNextBuffer=False, mode=3) :
    try:
        #Transmitting the 5 coeffs :
        motor.duration1 = duration
        motor.traj1_size = 5
        motor.a0_traj1 = polyCoeffs[0]
        motor.a1_traj1 = polyCoeffs[1]
        motor.a2_traj1 = polyCoeffs[2]
        motor.a3_traj1 = polyCoeffs[3]
        motor.a4_traj1 = polyCoeffs[4]
        motor.mode_dynaban = mode
        motor.send_mode_delayed = True
        motor.update_buffer1 = True
        if (copyNextBuffer) :
            #Enables the copy of traj2 into traj1 once traj1 finishes
            motor.send_copy_next_buffer_write = True
            motor.copy_next_buffer_write = 1
    except:
        print("Failed to set position polynom to motor id : ", id)
        
### Duration in tenth of ms (10000 = 1 sec)
def setTorqueTraj1ToMotor(motor, polyCoeffs) :
    try:
        #Transmitting the 5 coeffs :
        motor.torque1_size = 5
        motor.a0_torque1 = polyCoeffs[0]
        motor.a1_torque1 = polyCoeffs[1]
        motor.a2_torque1 = polyCoeffs[2]
        motor.a3_torque1 = polyCoeffs[3]
        motor.a4_torque1 = polyCoeffs[4]
        
        motor.update_buffer1 = True
    except:
        print("Failed to set torque polynom to motor id : ", id)
        
### Duration in tenth of ms (10000 = 1 sec)
def continueTrajWithMotor(motor, polyCoeffs, duration, mode=3) :
    try:
        #Transmitting the 5 coeffs :
        motor.duration2 = duration
        motor.traj2_size = 5
        motor.a0_traj2 = polyCoeffs[0]
        motor.a1_traj2 = polyCoeffs[1]
        motor.a2_traj2 = polyCoeffs[2]
        motor.a3_traj2 = polyCoeffs[3]
        motor.a4_traj2 = polyCoeffs[4]
        motor.send_mode_delayed = True
        motor.mode_dynaban = mode
        motor.update_buffer2 = True
        #Enables the copy of traj2 into traj1 once traj1 finishes
        motor.send_copy_next_buffer_write = True
        motor.copy_next_buffer_write = 1
            
    except:
        print("Failed to set position polynom to motor id : ", id)
        
def updateGoalPosition(motor):
    if (motor.mode_dynaban != 0) :
        #It would oscilate if mode == 0
        motor.goal_position = motor.present_position
    
def stopTrajectory(motor):
    print("Stopping traj for motor ", motor.id)
    motor.goal_position = motor.present_position
    motor.update_buffer1 = False
    motor.update_buffer2 = False
    motor.send_copy_next_buffer_write = True
    motor.copy_next_buffer_write = 0
    motor.send_mode = True
    motor.mode_dynaban = 0
    motor.send_mode_delayed = False
        
### Duration in tenth of ms (10000 = 1 sec)
def continueTorqueTraj1ToMotor(motor, polyCoeffs) :
    try:
        #Transmitting the 5 coeffs :
        motor.torque2_size = 5
        motor.a0_torque2 = polyCoeffs[0]
        motor.a1_torque2 = polyCoeffs[1]
        motor.a2_torque2 = polyCoeffs[2]
        motor.a3_torque2 = polyCoeffs[3]
        motor.a4_torque2 = polyCoeffs[4]
        
        motor.update_buffer2 = True
    except:
        print("Failed to set torque polynom to motor id : ", id)

def requestForceFromModel(params):
    inputStream = ""
    id = 1
    sign = 1
    for m in params.robot.motors :
        if id == 2 or id == 3 or id == 4:
            sign = -1
        else :
            sign = 1
        id = id + 1
        inputStream += str(m.present_position) + " " + str(sign*m.output_torque) + "\n"    

#     print inputStream
    
    command = "../Model/build/appTestLegTorques"
    arg = "force"
    process = Popen([command, arg], stdout=PIPE, stdin=PIPE)
    result = process.communicate(input=inputStream)[0]
 
#     print "Results = \n" + result
    output = result.split('\n')
    
    return output

def requestAntiGravityFromModel(params):
    inputStream = ""
    id = 1
    sign = 1
    for m in params.robot.motors :
        if id == 2 or id == 3 or id == 4:
            sign = -1
        else :
            sign = 1
        id = id + 1
        inputStream += str(m.present_position) + " " + str(0.0) + "\n"    

#     print inputStream
    
    command = "../Model/build/appTestLegTorques"
    arg = "gravity"
    process = Popen([command, arg], stdout=PIPE, stdin=PIPE)
    result = process.communicate(input=inputStream)[0]
 
#     print "Results = \n" + result
    output = result.split('\n')
    
    return output
    
def sign(x) :
    if (x > 0) :
        return 1
    if (x < 0) :
        return -1
    return 1

def isfloat(value):
  try:
    float(value)
    return True
  except:
    return False
