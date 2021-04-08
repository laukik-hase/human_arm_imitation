import time
import arm_control_utils

# offset = [180 180 90] # niryo
DXL_ID = [1,3,4,2]
my_arm_controller = arm_control_utils.arm_controller(DXL_ID)



my_arm_controller.initialize_motors()
my_arm_controller.enable_state_torque()

# replace this function with your torque control functions
my_arm_controller.set_goal_torque(DXL_ID[0],1023)
my_arm_controller.enable_torque_control_mode(DXL_ID[0])
#my_arm_controller.write_state([0])
time.sleep(0.1)
my_arm_controller.disable_torque_control_mode(DXL_ID[0])
# my_arm_controller.disable_state_torque()
my_arm_controller.stop_motors()