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
# *********     Read and Write Example      *********
#
#
# Available DXL model on this example : All models using Protocol 1.0
# This example is tested with a DXL MX-28, and an USB2DYNAMIXEL
# Be sure that DXL MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
#

import os

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




# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 1                 # Dynamixel ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 10           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 100            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

DURATION                    = 500
TRAJ_POLY1_SIZE             = 1
TRAJ_POLY1                  = [2048]
TORQUE_POLY1_SIZE           = 1
TORQUE_POLY1                = [5]


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

dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, DXL_ID)
if success(dxl_comm_result, dxl_error):
    print("Dynamixel connected successfully")

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
success(dxl_comm_result, dxl_error)

# set goal position
# dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, 2048)
# success(dxl_comm_result, dxl_error)


# Set trajectory duration
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_DURATION1, DURATION)
success(dxl_comm_result, dxl_error)

# Set trajectory size
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TRAJ_POLY1_SIZE, TRAJ_POLY1_SIZE)
success(dxl_comm_result, dxl_error)

# Set trajectory polynomial
for i in range(TRAJ_POLY1_SIZE):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_TRAJ_POLY1[i], TRAJ_POLY1[i])
    success(dxl_comm_result, dxl_error)

# Set torque size
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_POLY1_SIZE, TORQUE_POLY1_SIZE)
success(dxl_comm_result, dxl_error)

# Set torque polynomial
for i in range(TORQUE_POLY1_SIZE):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_POLY1[i], TORQUE_POLY1[i])
    success(dxl_comm_result, dxl_error)

# Set mode
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_MODE, 2)
success(dxl_comm_result, dxl_error)

# Disable Dynamixel Torque
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
# success(dxl_comm_result, dxl_error)

# Close port
portHandler.closePort()