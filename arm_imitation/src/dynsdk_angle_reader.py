import arm_control_utils
import time
DXL_ID = [3]
my_arm_controller = arm_control_utils.arm_controller(DXL_ID,devicename='/dev/ttyUSB0', baudrate=1000000)
my_arm_controller.initialize_motors()
# print(my_arm_controller.get_available_ports)
init_angle = my_arm_controller.get_present_position(DXL_ID[0])
print(init_angle)
time.sleep(1)
my_arm_controller.set_goal_position(DXL_ID[0], 2048)
time.sleep(2)
my_arm_controller.stop_motors()
