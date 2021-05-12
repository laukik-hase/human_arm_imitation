import arm_control_utils
import numpy as np
import sys
import time
import signal


DXL_ID = [1,4]
GRIPPER_ID = 5
JOINTS = len(DXL_ID)
TRANSFORMATION = [[-1,180], [1,180]]
RUN_MOTORS = 1
OPEN_LIMIT = 1023
CLOSE_LIMIT = 1023 - 160 # 170 steps stands for 50 degrees

def signal_handler(signal, frame):
    if RUN_MOTORS:
        print("Stopping dynamixel...")
        # my_arm_controller.disable_state_torque()
        # my_arm_controller.stop_motors()
    print("Goodbye!!!")
    sys.exit(0)

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

gripper_state = 0 # open

if RUN_MOTORS:
    my_arm_controller = arm_control_utils.arm_controller(DXL_ID,devicename='/dev/ttyUSB0', baudrate=1000000)
    my_arm_controller.initialize_motors()
    my_arm_controller.enable_torque(GRIPPER_ID)
    my_arm_controller.enable_torque(2)
    my_arm_controller.enable_torque(3)
    my_arm_controller.enable_state_torque()
    

csv_file = sys.argv[1]
data = np.genfromtxt(csv_file, delimiter=',')
angles = data[:,1:JOINTS+1]
gripper = data[:,-1]
at_init_pos = False
for i in range(len(gripper)):
    steps = angle_to_step(angles[i], TRANSFORMATION)
    print(steps)
    if RUN_MOTORS: 
        if not at_init_pos:
            go_to_start_pos(steps)
            gripper_move(0)
            at_init_pos = True
        else:
            my_arm_controller.write_state(steps)
            time.sleep(0.05)
            
    if gripper[i] != gripper_state:
        if RUN_MOTORS: gripper_move(gripper[i])
        gripper_state = gripper_state ^ 1
        print("gripper moved")

if RUN_MOTORS:
    my_arm_controller.disable_torque(GRIPPER_ID)
    my_arm_controller.disable_torque(2)
    my_arm_controller.disable_torque(3)
    my_arm_controller.disable_state_torque()
    my_arm_controller.stop_motors()