#!/usr/bin/env python

import numpy as np
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
ADDR_MX_TORQUE_LIMIT = 34       

DXL_MOVING_STATUS_THRESHOLD = 5            # Dynamixel moving status threshold


TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
COMM_SUCCESS = 0

MAX_YAW = 45
MAX_PITCH = 45

MAX_VEL_AZ = 300
MAX_VEL_EL = 100
TORQUE_LIMIT = 400

class HeadPoseController: 
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('head_pose_controller')
 
        # Initialize ROS
        rospy.Subscriber('/head_pose', Point, self.head_pose_callback, queue_size =1)

        pub = rospy.Publisher('current_head_pose', Point, queue_size=10)
        rate = rospy.Rate(10) # 10hz

        # Initialize PortHandler instances      
        self.portHandler1 = PortHandler(DEVICENAME1)
        self.portHandler2 = PortHandler(DEVICENAME2)

        # Initialize PacketHandler instance
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)


        self.watchdog = True
        
        self.dxl1_goal_position = 512 #initialyaw
        self.dxl2_goal_position = 600 #initial pitch

        self.dxl1_curr_position = 512 #initialyaw
        self.dxl2_curr_position = 600 #initial pitch



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
        
        # max pitch is +/-45 with zero at resting position (from datasheet)
        pitch = msg.y
        yaw = msg.z
        
        print(yaw)
        # map pitch and yaw degrees into RAM format
        
       # if(abs(pitch)>MAX_PITCH):
       #     pitch  = np.sign(pitch)*MAX_PITCH
       #     print("MAX PITCH reached")
       # if(abs(yaw)>MAX_YAW):
       #     yaw = np.sign(yaw)*MAX_YAW
       #     print("MAX YAW reached")
       # if(abs(pitch)<MAX_PITCH and abs(yaw)<MAX_YAW):
        self.dxl1_goal_position = int((yaw*512/150)+self.dxl1_curr_position)
        self.dxl2_goal_position = int((pitch*512/150)+self.dxl2_curr_position)

   #     print(f"From callback, yaw value actual is :{self.dxl1_curr_position}")
       # print(f"From callback, pitch value actual is :{150*self.dxl2_curr_position/512}")
        
        #print(f"From callback, pitch value transmitted is :{150*self.dxl2_goal_position/512}")

        print(f"From callback, yaw value actual is :{150*self.dxl1_curr_position/512}")

        print(f"From callback, yaw value goal is :{150*self.dxl1_goal_position/512}")


        if(abs(self.dxl1_goal_position-512)>512*MAX_YAW/150):
            print("max yaw reached")
            print()
            self.dxl1_goal_position = int(512 + np.sign(yaw)*512*MAX_YAW/150)

        if(abs(self.dxl2_goal_position-512)>512*MAX_PITCH/150):
            print("max pitch reached")
            print()
            self.dxl2_goal_position = int(512 + np.sign(pitch)*512*MAX_PITCH/150)

        # Write Dynamixel#1 goal position (yaw)
       # dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler1, DXL1_ID, ADDR_MX_GOAL_POSITION, self.dxl1_goal_position)
       # if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
       #     print("Failed to write goal position for Dynamixel#1")
#            self.watchdog = False

        # Write Dynamixel#2 goal position (pitch)
       # dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler2, DXL2_ID, ADDR_MX_GOAL_POSITION, self.dxl2_goal_position)
       # if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
       #     print("Failed to write goal position for Dynamixel#2")
#            self.watchdog = False
    
    def terminate(self):
        # Disable Dynamixel#1 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler1, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Disable Dynamixel#2 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler2, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Close port1
        self.portHandler1.closePort()

        # Close port2
        self.portHandler2.closePort()
        


if __name__ == "__main__":
scp -r user@your.server.example.com:/path/to/foo /home/user/Desktop/    head_controller = HeadPoseController()
    
    # Initialize ROS

    pub = rospy.Publisher('current_head_pose', Point, queue_size=10)
    rate = rospy.Rate(10) # 10hz


   
    if(head_controller.initialize()):
        # Write Dynamixel#1 goal position (yaw)
        dxl_comm_result, dxl_error = head_controller.packetHandler.write2ByteTxRx(head_controller.portHandler1, DXL1_ID, ADDR_MX_GOAL_POSITION, 512)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print("Failed to write goal position for Dynamixel#1")
            
        dxl_comm_result, dxl_error = head_controller.packetHandler.write2ByteTxRx(head_controller.portHandler2, DXL2_ID, ADDR_MX_GOAL_POSITION, 600)

        # Write Dynamixel#2 goal position (pitch)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print("Failed to write goal position for Dynamixel#2")

        # Write Dynamixel#1 max torque (yaw)
        dxl_comm_result, dxl_error = head_controller.packetHandler.write2ByteTxRx(head_controller.portHandler1, DXL1_ID, ADDR_MX_TORQUE_LIMIT, TORQUE_LIMIT)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print("Failed to write max torque for Dynamixel#1")

        dxl_comm_result, dxl_error = head_controller.packetHandler.write2ByteTxRx(head_controller.portHandler2, DXL2_ID, ADDR_MX_TORQUE_LIMIT, TORQUE_LIMIT)

        # Write Dynamixel#2 max torque (pitch)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print("Failed to write max torque for Dynamixel#2")


        while(head_controller.watchdog == True and not(rospy.is_shutdown())):
            while(head_controller.watchdog == True and not(rospy.is_shutdown())):
                # Write 
                dxl_comm_result, dxl_error = head_controller.packetHandler.write2ByteTxRx(head_controller.portHandler1, DXL1_ID, 32, MAX_VEL_AZ)
                dxl_comm_result, dxl_error = head_controller.packetHandler.write2ByteTxRx(head_controller.portHandler2, DXL2_ID, 32, MAX_VEL_EL)




                #read actual position servo1
                temp_yaw, dxl_comm_result, dxl_error = head_controller.packetHandler.read2ByteTxRx(head_controller.portHandler1, DXL1_ID, ADDR_MX_PRESENT_POSITION)
                if dxl_comm_result != COMM_SUCCESS:
                    print(head_controller.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print(head_controller.packetHandler.getRxPacketError(dxl_error))

                if(temp_yaw!=0):
                    head_controller.dxl1_curr_position = temp_yaw
                else:
                    print("0 in yaw, value not stored")

                #read actual position servo2
                temp_pitch, dxl_comm_result, dxl_error = head_controller.packetHandler.read2ByteTxRx(head_controller.portHandler2, DXL2_ID, ADDR_MX_PRESENT_POSITION)
                if dxl_comm_result != COMM_SUCCESS:
                    print(head_controller.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print(head_controller.packetHandler.getRxPacketError(dxl_error))
#!/usr/bin/env python

import numpy as np
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
ADDR_MX_TORQUE_LIMIT = 34       

DXL_MOVING_STATUS_THRESHOLD = 5            # Dynamixel moving status threshold


TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
COMM_SUCCESS = 0

MAX_YAW = 45
MAX_PITCH = 45

MAX_VEL_AZ = 300
MAX_VEL_EL = 100
TORQUE_LIMIT = 400

class HeadPoseController: 
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('head_pose_controller')
 
        # Initialize ROS
        rospy.Subscriber('/head_pose', Point, self.head_pose_callback, queue_size =1)

        pub = rospy.Publisher('current_head_pose', Point, queue_size=10)
        rate = rospy.Rate(10) # 10hz

        # Initialize PortHandler instances      
        self.portHandler1 = PortHandler(DEVICENAME1)
        self.portHandler2 = PortHandler(DEVICENAME2)

        # Initialize PacketHandler instance
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)


        self.watchdog = True
        
        self.dxl1_goal_position = 512 #initialyaw
        self.dxl2_goal_position = 600 #initial pitch

        self.dxl1_curr_position = 512 #initialyaw
        self.dxl2_curr_position = 600 #initial pitch



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
        
        # max pitch is +/-45 with zero at resting position (from datasheet)
        pitch = msg.y
        yaw = msg.z
        
        print(yaw)
        # map pitch and yaw degrees into RAM format
        
       # if(abs(pitch)>MAX_PITCH):
       #     pitch  = np.sign(pitch)*MAX_PITCH
       #     print("MAX PITCH reached")
       # if(abs(yaw)>MAX_YAW):
       #     yaw = np.sign(yaw)*MAX_YAW
       #     print("MAX YAW reached")
       # if(abs(pitch)<MAX_PITCH and abs(yaw)<MAX_YAW):
        self.dxl1_goal_position = int((yaw*512/150)+self.dxl1_curr_position)
        self.dxl2_goal_position = int((pitch*512/150)+self.dxl2_curr_position)

   #     print(f"From callback, yaw value actual is :{self.dxl1_curr_position}")
       # print(f"From callback, pitch value actual is :{150*self.dxl2_curr_position/512}")
        
        #print(f"From callback, pitch value transmitted is :{150*self.dxl2_goal_position/512}")

        print(f"From callback, yaw value actual is :{150*self.dxl1_curr_position/512}")

        print(f"From callback, yaw value goal is :{150*self.dxl1_goal_position/512}")


        if(abs(self.dxl1_goal_position-512)>512*MAX_YAW/150):
            print("max yaw reached")
            print()
            self.dxl1_goal_position = int(512 + np.sign(yaw)*512*MAX_YAW/150)

        if(abs(self.dxl2_goal_position-512)>512*MAX_PITCH/150):
            print("max pitch reached")
            print()
            self.dxl2_goal_position = int(512 + np.sign(pitch)*512*MAX_PITCH/150)

        # Write Dynamixel#1 goal position (yaw)
       # dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler1, DXL1_ID, ADDR_MX_GOAL_POSITION, self.dxl1_goal_position)
       # if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
       #     print("Failed to write goal position for Dynamixel#1")
#            self.watchdog = False

        # Write Dynamixel#2 goal position (pitch)
       # dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler2, DXL2_ID, ADDR_MX_GOAL_POSITION, self.dxl2_goal_position)
       # if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
       #     print("Failed to write goal position for Dynamixel#2")
#            self.watchdog = False
    
    def terminate(self):
        # Disable Dynamixel#1 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler1, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Disable Dynamixel#2 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler2, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Close port1
        self.portHandler1.closePort()

        # Close port2
        self.portHandler2.closePort()
        


if __name__ == "__main__":
scp -r user@your.server.example.com:/path/to/foo /home/user/Desktop/    head_controller = HeadPoseController()
    
    # Initialize ROS

    pub = rospy.Publisher('current_head_pose', Point, queue_size=10)
    rate = rospy.Rate(10) # 10hz


   
    if(head_controller.initialize()):
        # Write Dynamixel#1 goal position (yaw)
        dxl_comm_result, dxl_error = head_controller.packetHandler.write2ByteTxRx(head_controller.portHandler1, DXL1_ID, ADDR_MX_GOAL_POSITION, 512)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print("Failed to write goal position for Dynamixel#1")
            
        dxl_comm_result, dxl_error = head_controller.packetHandler.write2ByteTxRx(head_controller.portHandler2, DXL2_ID, ADDR_MX_GOAL_POSITION, 600)

        # Write Dynamixel#2 goal position (pitch)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print("Failed to write goal position for Dynamixel#2")

        # Write Dynamixel#1 max torque (yaw)
        dxl_comm_result, dxl_error = head_controller.packetHandler.write2ByteTxRx(head_controller.portHandler1, DXL1_ID, ADDR_MX_TORQUE_LIMIT, TORQUE_LIMIT)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print("Failed to write max torque for Dynamixel#1")

        dxl_comm_result, dxl_error = head_controller.packetHandler.write2ByteTxRx(head_controller.portHandler2, DXL2_ID, ADDR_MX_TORQUE_LIMIT, TORQUE_LIMIT)

        # Write Dynamixel#2 max torque (pitch)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print("Failed to write max torque for Dynamixel#2")


        while(head_controller.watchdog == True and not(rospy.is_shutdown())):
            while(head_controller.watchdog == True and not(rospy.is_shutdown())):
                # Write 
                dxl_comm_result, dxl_error = head_controller.packetHandler.write2ByteTxRx(head_controller.portHandler1, DXL1_ID, 32, MAX_VEL_AZ)
                dxl_comm_result, dxl_error = head_controller.packetHandler.write2ByteTxRx(head_controller.portHandler2, DXL2_ID, 32, MAX_VEL_EL)




                #read actual position servo1
                temp_yaw, dxl_comm_result, dxl_error = head_controller.packetHandler.read2ByteTxRx(head_controller.portHandler1, DXL1_ID, ADDR_MX_PRESENT_POSITION)
                if dxl_comm_result != COMM_SUCCESS:
                    print(head_controller.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print(head_controller.packetHandler.getRxPacketError(dxl_error))

                if(temp_yaw!=0):
                    head_controller.dxl1_curr_position = temp_yaw
                else:
                    print("0 in yaw, value not stored")

                #read actual position servo2
                temp_pitch, dxl_comm_result, dxl_error = head_controller.packetHandler.read2ByteTxRx(head_controller.portHandler2, DXL2_ID, ADDR_MX_PRESENT_POSITION)
                if dxl_comm_result != COMM_SUCCESS:
                    print(head_controller.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print(head_controller.packetHandler.getRxPacketError(dxl_error))

                if(temp_pitch!=0):
                    head_controller.dxl2_curr_position = temp_pitch
                else:
                    print("0 in pitch, value not stored")


                point = Point()
                point.z = head_controller.dxl1_curr_position
                point.y = head_controller.dxl2_curr_position
                
                pub.publish(point)



                # Write Dynamixel#1 goal position (yaw)
                dxl_comm_result, dxl_error = head_controller.packetHandler.write2ByteTxRx(head_controller.portHandler1, DXL1_ID, ADDR_MX_GOAL_POSITION, head_controller.dxl1_goal_position)
                if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                    print("Failed to write goal position for Dynamixel#1")
                #                   self.watchdog = False

                # Write Dynamixel#2 goal position (pitch)
                dxl_comm_result, dxl_error = head_controller.packetHandler.write2ByteTxRx(head_controller.portHandler2, DXL2_ID, ADDR_MX_GOAL_POSITION, head_controller.dxl2_goal_position)
                if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                    print("Failed to write goal position for Dynamixel#2")                

                print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL1_ID, head_controller.dxl1_goal_position, head_controller.dxl1_curr_position, DXL2_ID, head_controller.dxl2_goal_position, head_controller.dxl2_curr_position))



                if not ((abs(head_controller.dxl1_goal_position - head_controller.dxl1_curr_position) > DXL_MOVING_STATUS_THRESHOLD) and (abs(head_controller.dxl2_goal_position - head_controller.dxl2_curr_position) > DXL_MOVING_STATUS_THRESHOLD)):
                 #   print("end position reached")
                    break
            
            
        head_controller.terminate()



                point = Point()
                point.z = head_controller.dxl1_curr_position
                point.y = head_controller.dxl2_curr_position
                
                pub.publish(point)



                # Write Dynamixel#1 goal position (yaw)
                dxl_comm_result, dxl_error = head_controller.packetHandler.write2ByteTxRx(head_controller.portHandler1, DXL1_ID, ADDR_MX_GOAL_POSITION, head_controller.dxl1_goal_position)
                if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                    print("Failed to write goal position for Dynamixel#1")
                #                   self.watchdog = False

                # Write Dynamixel#2 goal position (pitch)
                dxl_comm_result, dxl_error = head_controller.packetHandler.write2ByteTxRx(head_controller.portHandler2, DXL2_ID, ADDR_MX_GOAL_POSITION, head_controller.dxl2_goal_position)
                if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                    print("Failed to write goal position for Dynamixel#2")                

                print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL1_ID, head_controller.dxl1_goal_position, head_controller.dxl1_curr_position, DXL2_ID, head_controller.dxl2_goal_position, head_controller.dxl2_curr_position))



                if not ((abs(head_controller.dxl1_goal_position - head_controller.dxl1_curr_position) > DXL_MOVING_STATUS_THRESHOLD) and (abs(head_controller.dxl2_goal_position - head_controller.dxl2_curr_position) > DXL_MOVING_STATUS_THRESHOLD)):
                 #   print("end position reached")
                    break
            
            
        head_controller.terminate()


