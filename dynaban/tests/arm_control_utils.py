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

ADDR_MX_TRAJ_POLY1_SIZE     = 0x4A 
ADDR_MX_TRAJ_POLY1          = [0x4B, 0x4F, 0x53, 0x57, 0x5B]
ADDR_MX_TORQUE_POLY1_SIZE   = 0x5F
ADDR_MX_TORQUE_POLY1        = [0x60, 0x64, 0x68, 0x6C, 0x70]
ADDR_MX_DURATION1           = 0X74             # duration is an integer in tenth of milliseconds (10000 is 1 s)  
ADDR_MX_TRAJ_POLY2_SIZE     = 0x76
ADDR_MX_TRAJ_POLY2          = [0x77, 0x7B, 0x7F, 0x83, 0x87]
ADDR_MX_TORQUE_POLY2_SIZE   = 0x8B
ADDR_MX_TORQUE_POLY2        = [0x8C, 0x90, 0x94, 0x98, 0x9C]
ADDR_MX_DURATION2           = 0XA0
ADDR_MX_MODE                = 0XA2  
ADDR_MX_COPY_NEXT_BUFFER    = 0XA3
ADDR_MX_POSITION_TRACKER_ON = 0XA4  
ADDR_MX_DEBUG_ON            = 0XA5
  

# Data Byte Length
LEN_MX_GOAL_POSITION       = 4
LEN_MX_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = [1] 
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 1024          # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 3072            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 5                # Dynamixel moving status threshold
MOVING_SPEED                = 100                           # Dynamixel moving speed  (0-1023)


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


JOINTS = len(DXL_ID)

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

def read_motor(_ID):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, _ID, ADDR_MX_PRESENT_POSITION)
    success(dxl_comm_result, dxl_error)
    return (dxl_present_position % 4096)

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
        _state[i] = read_motor(DXL_ID[i])
    return _state

def wait_until_goal_reached(_ID, _final_pos):
    while 1:
        # Read present position
        dxl_present_position = read_motor(_ID)
        if not (abs(_final_pos - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD):
            return

def set_position_trajectory(_ID, _DURATION, _TRAJ_POLY1, _TORQUE_POLY1):
    # Set trajectory duration
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, _ID, ADDR_MX_DURATION1, _DURATION)
    success(dxl_comm_result, dxl_error)

    # Set trajectory size
    _TRAJ_POLY1_SIZE = len(_TRAJ_POLY1)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, _ID, ADDR_MX_TRAJ_POLY1_SIZE, _TRAJ_POLY1_SIZE)
    success(dxl_comm_result, dxl_error)

    # Set torque size
    _TORQUE_POLY1_SIZE = len(_TORQUE_POLY1)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, _ID, ADDR_MX_TORQUE_POLY1_SIZE, _TORQUE_POLY1_SIZE)
    success(dxl_comm_result, dxl_error)
    
    # Set trajectory polynomial
    for i in range(_TRAJ_POLY1_SIZE):
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, _ID, ADDR_MX_TRAJ_POLY1[i], _TRAJ_POLY1[i])
        success(dxl_comm_result, dxl_error)
    
    # Set torque polynomial
    for i in range(_TORQUE_POLY1_SIZE):
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, _ID, ADDR_MX_TORQUE_POLY1[i], _TORQUE_POLY1[i])
        success(dxl_comm_result, dxl_error)



def set_mode(_ID, _MODE):    
    # Set mode
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, _ID, ADDR_MX_MODE, _MODE)
    success(dxl_comm_result, dxl_error)

def set_debug(_ID, _ON):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, _ID, ADDR_MX_DEBUG_ON, _ON)
    success(dxl_comm_result, dxl_error)


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