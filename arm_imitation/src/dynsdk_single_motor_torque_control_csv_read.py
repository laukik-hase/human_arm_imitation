import time
import arm_control_utils
import sys

def dxl_to_torque(value):
    return round(value / 10.23, 1)


def torque_to_dxl(value):
    return int(round(abs(value) * 10.23, 0))


def dxl_to_load(value):
    cw, load = divmod(value, 1024)
    direction = -2 * cw + 1

    return dxl_to_torque(load) * direction

# offset = [180 180 90] # niryo
DXL_ID = [1,3,2,4]
ID_SIZE = len(DXL_ID)
my_arm_controller = arm_control_utils.arm_controller(DXL_ID)
my_arm_controller.initialize_motors()
my_arm_controller.enable_state_torque()

#reading csv file
if ( len(sys.argv) == 2 ):
    file_name = sys.argv[1]
else:
    print("Usage: python csv_angle_publisher.py <csv_file_name>")
    exit()

with open(file_name, mode='r') as openfileobject:
    for line in openfileobject:
        str_state = line.split(',')
        value = float(str_state[1])
        new_value = torque_to_dxl(value)
#         new_value = dxl_to_load(value)
        print(new_value)
#         print(torque_to_dxl(new_value))
        my_arm_controller.set_goal_torque(DXL_ID[0],new_value)
        my_arm_controller.enable_torque_control_mode(DXL_ID[0])
        time.sleep(0.1)
        my_arm_controller.disable_torque_control_mode(DXL_ID[0])

my_arm_controller.disable_state_torque()
my_arm_controller.stop_motors()


# to control using hard coded values
#my_arm_controller.initialize_motors()
#my_arm_controller.enable_state_torque()
#my_arm_controller.set_goal_torque(DXL_ID[0],10)
#my_arm_controller.enable_torque_control_mode(DXL_ID[0])
#time.sleep(5)
#my_arm_controller.disable_torque_control_mode(DXL_ID[0])
#my_arm_controller.disable_state_torque()
#my_arm_controller.stop_motors()