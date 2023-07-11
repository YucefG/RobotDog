#!/usr/bin/env python

import os
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import rospy
from geometry_msgs.msg import Point


# Dynamixel constants
DXL1_ID = 4
DXL2_ID = 6
BAUDRATE = 1000000
DEVICENAME1 = "/dev/ttyUSB0"  # Replace with the correct port for Dynamixel#1
DEVICENAME2 = "/dev/ttyUSB0"  # Replace with the correct port for Dynamixel#2
PROTOCOL_VERSION = 1.0
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_PRESENT_POSITION   = 36

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
COMM_SUCCESS = 0


class HeadPoseController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('head_pose_controller')

        # Initialize PortHandler instances      
        self.portHandler1 = PortHandler(DEVICENAME1)
        self.portHandler2 = PortHandler(DEVICENAME2)

        # Initialize PacketHandler instance
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Initialize ROS
        rospy.Subscriber('/head_pose', Point, self.head_pose_callback)

        self.watchdog = True


        # Initialize PortHandler ins   
    def initialize(self):
        # Open port1 
        if self.portHandler1.openPort():
            print("Succeeded to open port1")
        else:
            print("Failed to open port1")
            return False

        # Open port2
        if self.portHandler2.openPort():
            print("Succeeded to open port2")
        else:
            print("Failed to open port2")
            return False

        # Set port1 baudrate
        if self.portHandler1.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate for port1")
        else:
            print("Failed to change the baudrate for port1")
            return False

        # Set port2 baudrate
        if self.portHandler2.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate for port2")
        else:
            print("Failed to change the baudrate for port2")
            return False

        # Enable Dynamixel#1 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler1, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print("Failed to enable torque for Dynamixel#1")
            return False

        # Enable Dynamixel#2 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler2, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print("Failed to enable torque for Dynamixel#2")
            return False


        return True



    def head_pose_callback(self, msg):
        pitch = msg.x
        yaw = msg.y
        #velocity = msg.z

        # transform degrees into steps

        # Write Dynamixel#1 goal position (yaw)
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(
            self.portHandler1, DXL1_ID, ADDR_MX_GOAL_POSITION, int(pitch)
        )
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print("Failed to write goal position for Dynamixel#1")
            self.watchdog = False

        # Write Dynamixel#2 goal position (pitch)
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(
            self.portHandler2, DXL2_ID, ADDR_MX_GOAL_POSITION, int(yaw)
        )
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print("Failed to write goal position for Dynamixel#2")
            self.watchdog = False

:

    def terminate(self):
        # Disable Dynamixel#1 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler1, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        # Disable Dynamixel#2 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler2, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        # Close port1
        portHandler1.closePort()

        # Close port2
        portHandler2.closePort()



if __name__ == "__main__":
    head_controller = HeadPoseController()
    if(head_controller.initialize()):
        while(head_controller.watchdog == True):
            rospy.spin()

        head_controller.terminate()

