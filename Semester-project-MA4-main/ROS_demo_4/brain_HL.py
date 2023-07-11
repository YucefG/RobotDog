###!/data/grebici/leg_env/bin/python

#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
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

AZIMUTH_FOV = 90
ELEVATION_FOV = 90



def deg2rad(deg):
    return deg*m.pi/180

class ImageProcessor:
    def __init__(self):
        rospy.init_node('brain_3')
        
        rospy.Subscriber('/audio_info_topic', String, self.callbackAudio)

     #   rospy.Subscriber('/camera/color/image_raw', Image, self.callbackFrame, callback_args=predictor)
        rospy.Subscriber('/video_info_topic', Frame_info_3, self.callbackFrame, queue_size=1)

        self.pub_hand = rospy.Publisher('/hand_pose', String, queue_size=1)
       # self.pub_pose = rospy.Publisher('/cmd_pose', Twist, queue_size=10)
        self.pub_mode = rospy.Publisher('/cmd_mode', String, queue_size=1)
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_head = rospy.Publisher('/head_pose', Point, queue_size=1)


        print(torch.cuda.is_available())
        print(torch.cuda.get_device_name())


        self.audioMemory = []
        self.generalMode = "undefined" #undefined, find_hum, gesture_command
        self.soundMode = 'undefined'  #modes : undefined, marvin, stop
        self.soundDirection = 'infront' #right or left
        self.personInFront = None
        self.poseDirection = None
        
        self.hand_action = "stay"

        self.tracking = True
        self.scanning = True

        self.command_elevation=0
        self.command_azimuth=0
        self.mem_elevation=0
        self.mem_azimuth=0
        self.timer = 0
        self.feasible_vel = True


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
        '''Returns true if human in front of camera''' #guiguiii
        left_shoulder = keypoints[self.posDict["left shoulder"]]
        right_shoulder = keypoints[self.posDict["right shoulder"]]

        
        mid_image = 0.5

        mid_shoulders = (left_shoulder.x+right_shoulder.x)/2 - mid_image
  #       # hard coded
        thresh = 0.1
      #  print(mid_shoulders)
        if abs(mid_shoulders) < thresh:
     #       print("found human in front!!")

            if(not(self.personInFront)):
                print("found human in front!!")
            return True
        else:
    #        print("PERSON NOT in front XXX")
            return False

    def keypoints2servo(self, keypoints):
        #value between 0 and 1. 
        left_shoulder = keypoints[self.posDict["left shoulder"]]
        right_shoulder = keypoints[self.posDict["right shoulder"]]

        mid_shoulders_x = (left_shoulder.x+right_shoulder.x)/2 -0.5
        mid_shoulders_y = (left_shoulder.y+right_shoulder.y)/2 -0.5

      #  print(f"azimuth is: {mid_shoulders_x*AZIMUTH_FOV}")
      #  print(f"elevation is:  {mid_shoulders_y*ELEVATION_FOV}")       
        #if near zero, azimuth and elevation near zero. Scaled by camera fov

        #keypoints are measured x-y errors from center of frame. 


        #sensed on actual frame
        azimuth = mid_shoulders_x*AZIMUTH_FOV   
        elevation = mid_shoulders_y*ELEVATION_FOV

        return azimuth, elevation
    
  
    def callbackFrame(self, data):
        print("frame callback")
        keypoints = data.keypoints
        hand_action = data.hand_action
 #       print(keypoints[12].x)
 #       print(keypoints[11].x)

        
        #measure time between two keypoint poses, to eject impossible bounds
        dt = time.perf_counter() - self.timer
        

        self.timer=time.perf_counter()

        self.personInFront =  self.person_in_front(keypoints)

        self.mem_azimuth = self.command_azimuth
        self.mem_elevation = self.command_elevation

        self.command_azimuth, self.command_elevation = self.keypoints2servo(keypoints)

        v_az = abs(self.mem_azimuth -self.command_azimuth)/dt
        v_el = abs(self.mem_elevation -self.command_elevation)/dt
        if(v_az > 10 or v_el>10):
            self.feasible_vel = False
        else:
            self.feasible_vel = True
        #print(self.feasible_vel)

        if(hand_action == 'sit' or 
            hand_action == 'down' or 
                hand_action == 'stay' or 
                    hand_action == 'come' or 
                        hand_action == 'hands_down'):
            self.hand_action = hand_action


        

    def callbackAudio(self, data):
       # print("IN AUDIOCALLBACK")
        data = str(data)[7:-1]  #remove the "data: " and the "'" at the end
        data = data.split("#")
        self.soundMode = data[0]
       # print("heard soundmode: "+self.soundMode)

        if self.soundMode == "marvin":
       #     print("changement de mode en mode find human (stop entendu)")
            self.soundDirection = data[1]
            self.audioMemory.append(self.soundDirection) # add new sound direction to memory
            self.audioMemory = self.audioMemory[:-10]
            self.generalMode = "find_human"
        if self.soundMode == "stop":
            print("changemende de mode en mode undefined (down entendu)")
            self.generalMode = "undefined"
        # for undefined sounds
        else:
            pass

        print("actual generalmode: "+self.generalMode)


        
    def info_to_vel(self):
        vel_msg = Twist()
        pose_msg = Twist()
        head_msg = Point()
        
      #  print("actual generalmode: "+self.generalMode)

        #if the previous mode was to find human, 
        #if(self.generalMode=="find_human"):
  #      print("waiting to receive audio order")
  #      rospy.wait_for_message("/video_info_topic", Frame_info_3, timeout=None)

        while(self.generalMode == 'find_human'):
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0 

            timer = time.perf_counter()

            # necessary? 
            counter = 0
            for elem in self.audioMemory[:-5]:
                    #         if elem == "center or unidentified":
                    counter += 1
            
            timer = time.perf_counter()

            if(self.personInFront):
                self.generalMode = 'hand_action'

            while(not(self.personInFront)):
                if(self.soundDirection == 'right'):
             #       print("turning RIGHT")
                    #vitesse de rotation negative
                    vel_msg.angular.z = -0.2

                elif(self.soundDirection=='left'):
            #        print("turning LEFT")
                    #vitesse de rotation positive
                    vel_msg.angular.z = 0.2

                elif counter == 5: 
                    print("movement finished BUT NO HUMAN FOUND")
                    vel_msg.angular.z = 0
                else:
             #       print('human behind')
                    vel_msg.angular.z = 0.2 
                
                 #        print(time.perf_counter() - timer)
                if(time.perf_counter() - timer)> SEARCHING_TIME:
                    print()
                    print("found nobody !")
                    self.generalMode='undefined'
                    print()
                    #set general mode to undefined -> robot at rest, stop searching
                    #do animation (camera left-right-left)
                    break
                
                time.sleep(0.05)
                self.pub_mode.publish(self.generalMode)
                self.pub_vel.publish(vel_msg)

            vel_msg.linear.x = 0
            vel_msg.linear.y = 0 
            vel_msg.angular.z = 0
            self.pub_vel.publish(vel_msg)


        if(self.generalMode == 'hand_action'): 
            
            # Process the image to determine direction

            print('----------------  hand_action !!! ------------------')
            #print("en commande vel par process de frames")
            # Create a Twist message with the appropriate velocity command
            #    print("waiting for camera info topic")
            #    rospyself.hand_action == "come".hand_action)

            if(self.hand_action == "sit"):
                self.pub_hand.publish('sit')
                
            elif(self.hand_action == "down"):
                self.pub_hand.publish('down')

            #elif(self.hand_action == "come"):
                #self.pub_hand.publish('come')
            
            elif(self.hand_action == "stay"):
                self.pub_hand.publish('stay')

            self.pub_mode.publish(self.generalMode)


        elif(self.generalMode == 'undefined'):
            #print("ROBOT AT REST")
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0 
            vel_msg.angular.z = 0

            error_az = abs(self.command_azimuth)
            error_el = abs(self.command_elevation)

            head_msg = Point()
            
            if(self.hand_action == "come"):
                
                if(error_az>10):
                    head_msg.z = -self.command_azimuth    

                #tout elevation > -10 est un faux positif (epaules plus bas que robot)
                if(error_el>10): #and self.command_elevation<0):
                    head_msg.y = self.command_elevation

                if(error_az>10 or error_el>10):
                    #en attendant qu'il arrete de bug
                    #head_msg.y = -30
                    print(head_msg)
                    time.sleep(0.1)
                    self.pub_head.publish(head_msg)            
                

            self.pub_vel.publish(vel_msg)
            self.pub_hand.publish('stay')

        # Publish the velocity command
        self.pub_mode.publish(self.generalMode)




  
if __name__ == '__main__':
    try:
        ip = ImageProcessor()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            ip.info_to_vel()
    except rospy.ROSInterruptException:
        pass

