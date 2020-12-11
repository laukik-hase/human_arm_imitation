#!/usr/bin/python

################################################################################

#!!! Code to write servo angle for different IDs simultaneously !!!

################################################################################

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

import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library

# Control table address
ADDR_MX_TORQUE_ENABLE       = 24                            # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION       = 30
ADDR_MX_PRESENT_POSITION    = 36
ADDR_MX_MOVING_SPEED        = 32

# Protocol version
PROTOCOL_VERSION            = 1                             # See which protocol version is used in the Dynamixel

# Default setting
BAUDRATE                    = 1000000
DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')# Check which port is being used on your controller
                                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 10                            # Dynamixel moving status threshold
MOVING_SPEED                = 200                           # Dynamixel moving speed  (0-1023)


COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed


# Set the port path
port_num = dynamixel.portHandler(DEVICENAME)

# Initialize PacketHandler Structs
dynamixel.packetHandler()

dxl_comm_result = COMM_TX_FAIL                              # Communication result

dxl_error = 0                                               # Dynamixel error
dxl_present_position = 0                                    # Present position

def enable_port():
    port_num = dynamixel.portHandler(DEVICENAME)
    
    # Open port
    if dynamixel.openPort(port_num):
        print("[+] Succeeded to open the port!")
    else:
        print("[-] Failed to open the port!")
        quit()

    # Set port baudrate
    if dynamixel.setBaudRate(port_num, BAUDRATE):
        print("[+] Succeeded to change the baudrate!")
    else:
        print("[-] Failed to change the baudrate!")
        quit()


def enable_bot(ID):
    # Enable Dynamixel Torque
    dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)
    dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)
    if dxl_comm_result != COMM_SUCCESS:
        print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
    elif dxl_error != 0:
        print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))
    else:
        print("[+] Dynamixel has been successfully connected")



def set_speed(ID):
    dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, ID, ADDR_MX_MOVING_SPEED, MOVING_SPEED)
    dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)
    dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)
    if dxl_comm_result != COMM_SUCCESS:
        print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
    elif dxl_error != 0:
        print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))


def write_value(ID, final_pos):

    # Write goal position
    dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, ID, ADDR_MX_GOAL_POSITION, final_pos)
    dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)
    dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)
    if dxl_comm_result != COMM_SUCCESS:
        print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
    elif dxl_error != 0:
        print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))


#Reading values of the Dynamixel ID

'''
    while 1:
        # Read present position
        dxl_present_position = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION, ID, ADDR_MX_PRESENT_POSITION)
        dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)
        dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)
        if dxl_comm_result != COMM_SUCCESS:
            print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
        elif dxl_error != 0:
            print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (ID, final_pos, dxl_present_position))

        if not (abs(final_pos - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD):
            break
'''

def disable_bot(ID):
    # Disable Dynamixel Torque
    dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)
    dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION)
'''    if dxl_comm_result != COMM_SUCCESS:
        print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
    elif dxl_error != 0:
        print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))
'''
    

if __name__ == '__main__':
    # Open port
    if dynamixel.openPort(port_num):
        print("[+] Succeeded to open the port!")
    else:
        print("[-] Failed to open the port!")
        quit()

    # Set port baudrate
    if dynamixel.setBaudRate(port_num, BAUDRATE):
        print("[+] Succeeded to change the baudrate!")
    else:
        print("[-] Failed to change the baudrate!")
        quit()
    try:
        theta0 = input("Enter Theta0 value: ")
        theta1 = input("Enter Theta1 value: ")
        theta2 = input("Enter Theta2 value: ")
        
        enable_bot(2)
        enable_bot(4)
        enable_bot(6)
        
        write_value(2, theta0)
        write_value(4, theta1)
        write_value(6, theta2)

        while 1:
            continue
    except KeyboardInterrupt:
        print("\n[+] Disabling")
        disable_bot(2)
        disable_bot(4)
        disable_bot(6)    
        
        # Close port
        dynamixel.closePort(port_num)