###!/data/grebici/leg_env/bin/python

#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point
import cv2
from cv_bridge import CvBridge
import numpy as np
import PIL
import torch
from custom_msg.msg import Frame_info_3
import numpy as np
import time
from std_msgs.msg import String
import math as m

ROLL_MAX = 15
PITCH_MAX = 15
SEARCHING_TIME = 10
ROTATION_SPEED = 0.2
MAX_HEAD_YAW_ANGLE = 30 # in degrees
SEARCHING_TIME = 10 # in seconds
DIST_CAM_USER = 2 # in m
DIST_ROT_GO1_CAM = 0.2 # in m
AZIMUTH_FOV = 90 # in degrees
ELEVATION_FOV = 90 # in degrees

AZIMUTH_MAX = 30
AZIMUTH_MIN = -30

ELEV_MAX = 45
ELEV_MIN = 0

def deg2rad(deg):
    return deg*m.pi/180

def rad2deg(deg):
    return deg*180/m.pi

def serv2deg(stepCounter):
    return (stepCounter/1024) * 300 - 150

def camAngle2Go1Angle(camAngle, depth=DIST_CAM_USER):
    '''
    camAngle btwn 0 and 1023 to represent bwtn 0 and 300
    '''
    camAngle = deg2rad((camAngle/1024) * 300 - 150)
    return np.arctan2((depth*np.sin(camAngle)),(DIST_ROT_GO1_CAM+depth*np.cos(camAngle)))

def go1Angle2movingTime(go1Angle , speedRota=0.2):
    '''
    angle in rad, speedRota in rad/s, output in s
    '''
    return abs(go1Angle/speedRota)

class ImageProcessor:
    def __init__(self):
        # create brain node
        rospy.init_node('brain_4')
        
        # subscribe to audio and video topics
        rospy.Subscriber('/audio_info_topic', String, self.callbackAudio)
        rospy.Subscriber('/video_info_topic', Frame_info_3, self.callbackFrame, queue_size=1)
        rospy.Subscriber('/current_head_pose', Point, self.callbackHead, queue_size=1)

    
        # create publishers
        self.pub_hand = rospy.Publisher('/hand_pose', String, queue_size=1)
        self.pub_mode = rospy.Publisher('/cmd_mode', String, queue_size=1)
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_head = rospy.Publisher('/head_pose', Point, queue_size=1)
        self.pub_pose = rospy.Publisher('/cmd_pose', Twist, queue_size=1)



        # init main variables
        self.generalMode = "off" #general modes: off, bed, find_fuman, dog, visual, happy
        self.soundMode = "off"  #keyword recieved: undefined, off, bed, marvin, dog, visual, happy
        self.soundDirection = "infront" #right or left
        self.personInFront = None
        self.poseDirection = None
        self.followFlag = True
        self.hand_action = "none"
        self.hand_action_buffer  = "none"

        self.command_elevation=0        #in degrees
        self.command_azimuth=0          #in degrees
        self.mem_elevation=0            #in degrees
        self.mem_azimuth=0              #in degrees
        self.timer = 0
        self.feasible_vel = True      
        self.current_elevation = 600    #in tics
        self.current_azimuth = 512      #in tics

        self.posDict = {"nose":0,
                    "left eye (inner)":1,
                    "left eye":2,
                    "left eye (outer)":3,
                    "right eye (inner)":4,
                    "right eye":5,
                    "right eye (outer)":6,
                    "left ear":7,
                    "right ear":8,
                    "mouth (left)":9,
                    "mouth (right)":10,
                    "left shoulder":11,
                    "right shoulder":12,
                    "left elbow":13,
                    "right elbow":14,
                    "left wrist":15,
                    "right wrist":16,
                    "left pinky":17,
                    "right pinky":18,
                    "left index":19,
                    "right index":20,
                    "left thumb":21,
                    "right thumb":22,
                    "left hip":23,
                    "right hip":24,
                    "left knee":25,
                    "right knee":26,
                    "left ankle":27,
                    "right ankle":28,
                    "left heel":29,
                    "right heel":30,
                    "left foot index":31,
                    "right foot index":32}
        
    
    def person_in_front(self, keypoints): 
        '''Returns true if human in front of camera''' 
        left_shoulder = keypoints[self.posDict["left shoulder"]]
        right_shoulder = keypoints[self.posDict["right shoulder"]]

        mid_image = 0.5

        mid_shoulders = (left_shoulder.x+right_shoulder.x)/2 - mid_image
        thresh = 0.1
        if abs(mid_shoulders) < thresh:
            if(not(self.personInFront)):
                print("found human in front!!")
            return True
        else:
            return False


    def callbackFrame(self, data):
        '''Function called each time new data is recived from the video node
        passes the keypoint to the person_in_front() function to check if someone is in the center of the frame
        modifies the self.hand_action if conditions are met'''
        keypoints = data.keypoints
        hand_action = data.hand_action
        self.personInFront =  self.person_in_front(keypoints)

        self.mem_azimuth = self.command_azimuth
        self.mem_elevation = self.command_elevation

        self.command_azimuth, self.command_elevation = self.keypoints2servo(keypoints)        
  
        if(hand_action == "sit" or hand_action == "down" or hand_action == "stay" or hand_action == "come"):
            if(hand_action == "down"):
                if(self.hand_action_buffer=="down"):
                    self.hand_action = hand_action
                else:
                    self.hand_action_buffer= hand_action
            else:
                self.hand_action = hand_action
        else:
            self.hand_action = None # no adquate hand singal found
            # pass # reuse the last action
    #    print(f"general mode is: {self.generalMode}")
    #    print(f"audio mode is: {self.soundMode}")


    def callbackHead(self, data):
        #print(f"azimuth callback value: {data.z}")
        self.current_azimuth = data.z
        self.current_elevation = data.y


    def callbackAudio(self, data):
        '''Function called when new data is recieved from the audi node
        changes the generlMode if a particular keyword is heard
        can also sent the direction of the sound if needed'''
        
        data = str(data)[7:-1]  #remove the "data: " and the "'" at the end
        data = data.split("#")
        self.soundMode = data[0]

        if self.soundMode == "marvin": # will try to find the human
            self.soundDirection = data[1]
            self.generalMode = "find_human"
        
        if self.soundMode == "off" or self.soundMode == "stop": # just stay and do nothing
            self.generalMode = "off"
        
        if self.soundMode == "bed": # lay down and do nothing
            self.generalMode = "bed"
        
        if self.soundMode == "dog": # multimodal live ineraction
            self.generalMode = "dog"
        
        if self.soundMode == "visual": # visual only live interaction
            self.generalMode = "visual"
        
        if self.soundMode == "happy": # hard coded interaction
            self.generalMode = "happy"

        if self.soundMode == "follow": # toggle follow flag
            self.followFlag = not(self.followFlag)
            print("follow flag toggled to: " + str(self.followFlag))

        else: # for undefined sounds
            pass

        print("actual generalmode in audio callback function:" + self.generalMode)

    def keypoints2servo(self, keypoints):
        #value between 0 and 1. 
        left_shoulder = keypoints[self.posDict["left shoulder"]]
        right_shoulder = keypoints[self.posDict["right shoulder"]]

        mid_shoulders_x = (left_shoulder.x+right_shoulder.x)/2 - 0.5
        mid_shoulders_y = (left_shoulder.y+right_shoulder.y)/2 - 0.5

        #if near zero, azimuth and elevation near zero. Scaled by camera fov

        #keypoints are measured x-y errors from center of frame. 


        #sensed on actual frame
        azimuth = mid_shoulders_x*AZIMUTH_FOV   
        elevation = -mid_shoulders_y*ELEVATION_FOV

       # print(f"azimuth is: {azimuth}")
       # print(f"elevation is:  {elevation}")       

        return azimuth, elevation


    def head_tracking(self):
        head_msg = Point()
        error_az = abs(self.command_azimuth)
        error_el = abs(self.command_elevation)

        if(self.followFlag):
            actual_elevation = serv2deg(self.current_elevation) 
            actual_azimuth = serv2deg(self.current_azimuth)

       #     print(f"Actual elevation: {actual_elevation}")
      #      print(f"command elevation: {self.command_elevation}")

     #       print(f"target elevation : {actual_elevation + self.command_elevation}")

    #        print(f"Actual azimuth: {actual_azimuth}")
    #        print(f"command azimuth: {self.command_azimuth}")
    #        print(f"target azimuth: {actual_azimuth + self.command_azimuth}")

            
            '''
            if((actual_azimuth + self.command_azimuth)>AZIMUTH_MAX):
                print("max azimuth is reached")

            elif((actual_azimuth + self.command_azimuth)<AZIMUTH_MIN):
                print("max azimuth is reached")
                
            elif((actual_elevation + self.command_elevation)>ELEV_MAX):
                print("max elevation is reached")
                
            elif((actual_elevation + self.command_elevation)<AZIMUTH_MIN):
                print("min elevation is reached")
            ''' 

            if(error_az>5 or error_el>10):
                if(error_az>10):
                    head_msg.z = self.command_azimuth 
                if(error_el>10):
                    head_msg.y = self.command_elevation 
                if(actual_azimuth + head_msg.z>AZIMUTH_MAX):
                    if(abs(AZIMUTH_MAX - actual_azimuth)>5):
                #        print("max azimuth is reached")
                        head_msg.z = AZIMUTH_MAX - actual_azimuth
                    else: 
                        head_msg.z =0 
                if(actual_azimuth + head_msg.z<AZIMUTH_MIN):
                    if(abs(AZIMUTH_MIN - actual_azimuth)>5):
                  #      print("min azimuth is reached")
                        head_msg.z = AZIMUTH_MIN - actual_azimuth
                    else: 
                        head_msg.z =0 
               #     print(f"sent az is: {head_msg.z}")

                if(actual_elevation + head_msg.y > ELEV_MAX):
                    if(abs(ELEV_MAX - actual_elevation)>5):
                   #     print("max elevation is reached")
                        head_msg.y = ELEV_MAX - actual_elevation
                    else: 
                        head_msg.y=0 
                if(actual_elevation + head_msg.y < ELEV_MIN):
                    if(abs(ELEV_MIN - actual_elevation)>5):
                    #    print("min elevation is reached")
                        head_msg.y = ELEV_MIN - actual_elevation
                    else: 
                        head_msg.y=0 
        else: 
            if(abs(serv2deg(self.current_azimuth))>5):
                head_msg.z = -serv2deg(self.current_azimuth)
            if(abs(serv2deg(self.current_elevation))>10):
                head_msg.y = -serv2deg(self.current_elevation)


        self.pub_head.publish(head_msg)
   #     print(head_msg)



    def align_head_with_body(self, head_yaw_angle):
        # read last neck yaw angle and transform it in 
        vel_msg = Twist()        
        rota_time = go1Angle2movingTime(camAngle2Go1Angle(self.current_azimuth), ROTATION_SPEED)
        timer=0
        vel_msg.angular.z = np.sign((self.current_azimuth/1024) * 300 - 150)*ROTATION_SPEED
        print(f"angle in tics of az of camera is : {self.current_azimuth}")
        print(f"angle in degrees of az of camera is : {(self.current_azimuth/1024) * 300 - 150}")
        print(vel_msg)

 
        print(f"angle of body is : {rad2deg(camAngle2Go1Angle(self.current_azimuth))}")
        print(f"rotation time is : {rota_time}")
        while timer<rota_time:
            timer += 0.05
            time.sleep(0.05)
            self.pub_mode.publish("center_human")
            self.pub_vel.publish(vel_msg)
            print()
            print(timer)
            print(vel_msg)
            self.head_tracking()

            
        self.generalMode = 'off'


    def info_to_vel(self):
        '''sends commands to the control node based on current audio and video states
        reacts differently based on the desired mode'''
        vel_msg = Twist()
        pose_msg = Twist()
        head_msg = Point()

    #    print("general mode is: ", self.generalMode)
        
        #### FIND_HUMAN MODE ####
        while(self.generalMode == 'find_human'):
            sound_direction = self.soundDirection
            self.pub_mode.publish(self.generalMode)
            head_yaw_angle = 0
            if sound_direction == "right":
                head_yaw_angle = -MAX_HEAD_YAW_ANGLE
            #    print(serv2deg(self.current_azimuth))
            #    print(head_yaw_angle)
                head_msg.z = head_yaw_angle - serv2deg(self.current_azimuth)
                head_msg.y = 25 - serv2deg(self.current_elevation)
            #    print("publishing head right")
            #    print(head_msg)

                self.pub_head.publish(head_msg)
            elif sound_direction == "left":
                head_yaw_angle = MAX_HEAD_YAW_ANGLE
                head_msg.z = head_yaw_angle - serv2deg(self.current_azimuth)
            #    print(serv2deg(self.current_azimuth))
            #    print(head_yaw_angle)
                head_msg.y = 25 - serv2deg(self.current_elevation)
                print("publishing head left")
           #     print(head_msg)

                self.pub_head.publish(head_msg)
            elif not(self.personInFront):
                head_yaw_angle = -MAX_HEAD_YAW_ANGLE
                head_msg.z = head_yaw_angle - serv2deg(self.current_azimuth)
            #    print(serv2deg(self.current_azimuth))
            #    print(head_yaw_angle)
                head_msg.y = 25 - serv2deg(self.current_elevation)
                print("publishing head left because behind")
                self.pub_head.publish(head_msg)

            print(self.personInFront)
            if self.personInFront:
            #    self.generalMode = 'off'
                self.pub_mode.publish("center_human")

                print("found human just by turning head")
                self.align_head_with_body(head_yaw_angle)
            else:
                timer = 0
            #    vel_msg.angular.z = np.sign(head_yaw_angle)*0.5
                print("looking for human by moving body")
                while (timer<SEARCHING_TIME and self.generalMode == 'find_human'):
                    print(sound_direction)
          
                    if sound_direction == "left":
                        vel_msg.angular.z = 0.5
                    else:
                        vel_msg.angular.z =-0.5
                    self.pub_vel.publish(vel_msg)
                    timer += 0.05
                    time.sleep(0.05)

                    if self.personInFront:
                    #    self.generalMode = 'off'
                        print("found human by moving body")
                        self.align_head_with_body(head_yaw_angle)
                        break
                    else: 
                        print("did not find human by moving body")
                self.generalMode = 'off'
           #     print("did not find human")

        #### VISUAL ONLY MODE ####
        if(self.generalMode == 'visual'): 
            self.pub_mode.publish(self.generalMode)

            #       print('----------------  hand_action !!! ------------------')
            print(self.hand_action)
            print()

            if(self.hand_action == "sit"):
                self.pub_hand.publish('sit')
                
            elif(self.hand_action == "down"):
                self.pub_hand.publish('down')
            
            elif(self.hand_action == "stay"):
                self.pub_hand.publish('stay')

            # elif(self.hand_action == "come" and self.soundMode == "follow"):
            #     self.followFlag = not(self.followFlag) # toggle the follow flag
            #     print("follow flag: " + str(self.followFlag))
            #     self.pub_hand.publish('come')
            self.head_tracking()

        #### DOG MODE ####
        if (self.generalMode == 'dog'):
            self.pub_mode.publish(self.generalMode)

#            print('----------------  hand_action !!! ------------------')
#            print(self.hand_action)

            if(self.hand_action == "sit" and self.soundMode == "six"):
                print("sit")
                self.pub_hand.publish('sit')
                
            elif(self.hand_action == "down" and self.soundMode == "down"):
                print("down")
                self.pub_hand.publish('down')
            
            elif(self.hand_action == "stay" and self.soundMode == "up"):
                print("stay")
                self.pub_hand.publish('stay')

            if(not(self.hand_action == "down" and self.soundMode == "down") and 
                not(self.hand_action == "stay" and self.soundMode == "up") and
                not(self.hand_action == "sit" and self.soundMode == "six")and 
                (self.hand_action == "down" or self.hand_action == "stay" or self.hand_action == "sit") and 
                (self.soundMode == "down" or self.soundMode == "up" or self.soundMode == "six")):
                print("incomprehension")
                self.pub_hand.publish('incomprehension')
                time.sleep(1)
                self.pub_hand.publish('stay')
            
            self.head_tracking()



            # elif(self.hand_action == "come" and self.soundMode == "follow"):
            #     self.followFlag = not(self.followFlag) # toggle the follow flag
            #     print("follow flag: " + str(self.followFlag))
            #     self.pub_hand.publish('follow')


        #### HAPPY MODE ####
        if(self.generalMode == 'happy'):
            happy_timer = 0
            mouv_cntr = 0
            print("in  happy")

            while(self.generalMode == 'happy'):
                print("in while happy")


                if(mouv_cntr==5):
                    mouv_cntr=0

                if(mouv_cntr==0):
                    pose_msg = Twist()
                    pose_msg.angular.y = deg2rad(20) 
                    self.pub_pose.publish(pose_msg)
                if(mouv_cntr==1):
                    pose_msg = Twist()
                    pose_msg.angular.y = deg2rad(-10)
                    self.pub_pose.publish(pose_msg)

                if(mouv_cntr==2):
                    pose_msg = Twist()
                    pose_msg.angular.y = deg2rad(20)
                    self.pub_pose.publish(pose_msg)

                if(mouv_cntr==3):
                    pose_msg = Twist()
                    pose_msg.angular.y = deg2rad(-10)
                    self.pub_pose.publish(pose_msg)

                if(mouv_cntr==4):
                    pose_msg = Twist()
                    pose_msg.angular.y = deg2rad(20)
                    self.pub_pose.publish(pose_msg)

                self.pub_mode.publish(self.generalMode)
                time.sleep(0.7)
                happy_timer += 0.7
                mouv_cntr += 1

               # self.head_tracking()



        #### BED MODE ####
        if(self.generalMode == 'bed'):
            self.pub_mode.publish(self.generalMode)
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0 
            vel_msg.angular.z = 0
            self.pub_vel.publish(vel_msg)
            self.pub_hand.publish('down')

            self.head_tracking()


        #### OFF MODE ####
        if(self.generalMode == 'off'):
         #   print("in off")
            self.pub_mode.publish(self.generalMode)

            vel_msg.linear.x = 0
            vel_msg.linear.y = 0 
            vel_msg.angular.z = 0
            self.pub_vel.publish(vel_msg)
            self.pub_hand.publish('stay')
        
            self.head_tracking()


        self.pub_mode.publish(self.generalMode)

   
if __name__ == '__main__':

    try:
        ip = ImageProcessor()
        while not rospy.is_shutdown():
            ip.info_to_vel()
    except rospy.ROSInterruptException:
        pass

