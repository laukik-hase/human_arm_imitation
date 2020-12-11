#!/usr/bin/env python

import os
import time
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


# Control table address
ADDR_MX_TORQUE_ENABLE       = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION       = 30
ADDR_MX_PRESENT_POSITION    = 36
ADDR_MX_MOVING_SPEED        = 32

# Data Byte Length
LEN_MX_GOAL_POSITION       = 4
LEN_MX_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = [1, 2, 8] 
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 1024          # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 3072            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
MOVING_SPEED                = 100                           # Dynamixel moving speed  (0-1023)

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)

# Initialize GroupBulkRead instace for Present Position
groupBulkRead = GroupBulkRead(portHandler, packetHandler)

# traj1 = [range(dxl_goal_position[0], dxl_goal_position[1]+1, 4),
#         range(dxl_goal_position[0], dxl_goal_position[1]+1, 4),
#         range(dxl_goal_position[0], dxl_goal_position[1]+1, 4)]
# traj2 = [range(dxl_goal_position[1], dxl_goal_position[0]-1, -4),
#         range(dxl_goal_position[1], dxl_goal_position[0]-1, -4),
#         range(dxl_goal_position[1], dxl_goal_position[0]-1, -4)]

JOINTS = 3



def success(_dxl_comm_result, _dxl_error):
    status = False
    if _dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(_dxl_comm_result))
    elif _dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(_dxl_error))
    else:
        status = True
    return status

def set_speed(ID):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, ID, ADDR_MX_MOVING_SPEED, MOVING_SPEED)
    if success(dxl_comm_result, dxl_error):
        print("Speed set successfully for Dynamixel#%d" % ID)

def enable_torque(ID):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if success(dxl_comm_result, dxl_error):
        print("Torque enabled successfully for Dynamixel#%d" % ID)

def disable_torque(ID):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if success(dxl_comm_result, dxl_error):
        print("Torque disabled successfully for Dynamixel#%d" % ID)

def enable_state_torque():
    # Enable Dynamixel Torque
    for i in range(JOINTS):
        enable_torque(DXL_ID[i])
    
def disable_state_torque():
    # Disable Dynamixel Torque
    for i in range(JOINTS):
        disable_torque(DXL_ID[i])

def to_byte_array(value):
    return [DXL_LOBYTE(DXL_LOWORD(value)), DXL_HIBYTE(DXL_LOWORD(value)), DXL_LOBYTE(DXL_HIWORD(value)), DXL_HIBYTE(DXL_HIWORD(value))]

def write_state(_state):
    for i in range(JOINTS): 
        # Allocate goal position value into byte array
        param_goal_position = to_byte_array(_state[i])

        # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[i], param_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID[i])
            quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

    # time.sleep(0.01)

def read_state():
    _state = [-1 for i in range(JOINTS)]
    for i in range(JOINTS):
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID[i], ADDR_MX_PRESENT_POSITION)
        success(dxl_comm_result, dxl_error)
        _state[i] = dxl_present_position % 4096
    return _state

def initialize_motors():
    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

def stop_motors():
    # Close port
    portHandler.closePort()