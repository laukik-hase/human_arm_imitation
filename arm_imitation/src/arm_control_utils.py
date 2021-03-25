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

class arm_controller:
    def __init__(self, dxl_id, devicename='/dev/ttyUSB0', baudrate=1000000):

        # Control table address
        self.ADDR_MX_TORQUE_ENABLE       = 24               # Control table address is different in Dynamixel model
        self.ADDR_MX_GOAL_POSITION       = 30
        self.ADDR_MX_PRESENT_POSITION    = 36
        self.ADDR_MX_MOVING_SPEED        = 32

        # Data Byte Length
        self.LEN_MX_GOAL_POSITION       = 4
        self.LEN_MX_PRESENT_POSITION    = 4

        # Protocol version
        self.PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

        # Default setting
        self.DXL_ID                      = dxl_id
        self.JOINTS                      = len(self.DXL_ID)
        self.BAUDRATE                    = baudrate             # Dynamixel default baudrate : 57600
        self.DEVICENAME                  = devicename    # Check which port is being used on your controller
                                                        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        self.DXL_MINIMUM_POSITION_VALUE  = 1024          # Dynamixel will rotate between this value
        self.DXL_MAXIMUM_POSITION_VALUE  = 3072            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        self.DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
        self.MOVING_SPEED                = 100                           # Dynamixel moving speed  (0-1023)
        # self.OFFSETS                     = offsets
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self.DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Initialize GroupSyncWrite instance
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_MX_GOAL_POSITION, self.LEN_MX_GOAL_POSITION)

        # Initialize GroupBulkRead instace for Present Position
        self.groupBulkRead = GroupBulkRead(self.portHandler, self.packetHandler)


    def success(self, _dxl_comm_result, _dxl_error):
        status = False
        if _dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(_dxl_comm_result))
        elif _dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(_dxl_error))
        else:
            status = True
        return status

    def set_speed(self, ID):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, self.ADDR_MX_MOVING_SPEED, self.MOVING_SPEED)
        if self.success(dxl_comm_result, dxl_error):
            print("Speed set self.successfully for Dynamixel#%d" % ID)

    def enable_torque(self, ID):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if self.success(dxl_comm_result, dxl_error):
            print("Torque enabled self.successfully for Dynamixel#%d" % ID)

    def disable_torque(self, ID):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if self.success(dxl_comm_result, dxl_error):
            print("Torque disabled self.successfully for Dynamixel#%d" % ID)

    def enable_state_torque(self):
        # Enable Dynamixel Torque
        for i in range(self.JOINTS):
            self.enable_torque(self.DXL_ID[i])
        
    def disable_state_torque(self):
        # Disable Dynamixel Torque
        for i in range(self.JOINTS):
            self.disable_torque(self.DXL_ID[i])

    def to_byte_array(self, value):
        return [DXL_LOBYTE(DXL_LOWORD(value)), DXL_HIBYTE(DXL_LOWORD(value)), DXL_LOBYTE(DXL_HIWORD(value)), DXL_HIBYTE(DXL_HIWORD(value))]

    def write_state(self, _state):
        for i in range(self.JOINTS): 
            # Allocate goal position value into byte array
            param_goal_position = self.to_byte_array(_state[i])

            # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
            dxl_addparam_result = self.groupSyncWrite.addParam(self.DXL_ID[i], param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] self.groupSyncWrite addparam failed" % self.DXL_ID[i])
                quit()

        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()

        # time.sleep(0.01)

    def read_state(self):
        _state = [-1 for i in range(self.JOINTS)]
        for i in range(self.JOINTS):
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID[i], self.ADDR_MX_PRESENT_POSITION)
            self.success(dxl_comm_result, dxl_error)
            _state[i] = dxl_present_position % 4096
        return _state

    def initialize_motors(self):
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

    def stop_motors(self):
        # Close port
        self.portHandler.closePort()