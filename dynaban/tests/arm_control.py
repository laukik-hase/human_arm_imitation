#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Sync Write Example      *********
#
#
# Available Dynamixel model on this example : All models using Protocol 1.0
# This example is tested with two Dynamixel MX-28, and an USB2DYNAMIXEL
# Be sure that Dynamixel MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
#

import os
import time

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

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

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

# traj1 = [range(dxl_goal_position[0], dxl_goal_position[1]+1, 4),
#         range(dxl_goal_position[0], dxl_goal_position[1]+1, 4),
#         range(dxl_goal_position[0], dxl_goal_position[1]+1, 4)]
# traj2 = [range(dxl_goal_position[1], dxl_goal_position[0]-1, -4),
#         range(dxl_goal_position[1], dxl_goal_position[0]-1, -4),
#         range(dxl_goal_position[1], dxl_goal_position[0]-1, -4)]

JOINTS = 3

traj1 = [[i for j in range(JOINTS)] for i in range(dxl_goal_position[0], dxl_goal_position[1]+1, 4)]
traj2 = [[i for j in range(JOINTS)] for i in range(dxl_goal_position[1], dxl_goal_position[0]-1, -4)]



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

def to_byte_array(value):
    return [DXL_LOBYTE(DXL_LOWORD(value)), DXL_HIBYTE(DXL_LOWORD(value)), DXL_LOBYTE(DXL_HIWORD(value)), DXL_HIBYTE(DXL_HIWORD(value))]

def follow_trajectory(trajectory):
    for state in trajectory:
        for i in range(JOINTS): 
            # Allocate goal position value into byte array
            param_goal_position = to_byte_array(state[i])

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

        time.sleep(0.01)

# Enable Dynamixel Torque
for i in range(len(DXL_ID)):
    enable_torque(DXL_ID[i])

# Set speed
# for i in range(len(DXL_ID)):
#     set_speed(DXL_ID[i])

while 1:
    follow_trajectory(traj1)
    follow_trajectory(traj2)


for i in range(len(DXL_ID)):
    disable_torque(DXL_ID[i])

# Close port
portHandler.closePort()