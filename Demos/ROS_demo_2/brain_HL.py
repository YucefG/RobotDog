###!/data/grebici/leg_env/bin/python

#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np
import PIL
import torch
from custom_msg.msg import Frame_info
import numpy as np
import time
from std_msgs.msg import String
import math as m

ROLL_MAX = 15
PITCH_MAX = 15
SEARCHING_TIME = 10

def deg2rad(deg):
    return deg*m.pi/180

class ImageProcessor:
    def __init__(self):
        rospy.init_node('image_processor')
        
        rospy.Subscriber('/audio_info_topic', String, self.callbackAudio)

     #   rospy.Subscriber('/camera/color/image_raw', Image, self.callbackFrame, callback_args=predictor)
        rospy.Subscriber('/video_info_topic', Frame_info, self.callbackFrame)

        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_pose = rospy.Publisher('/cmd_pose', Twist, queue_size=10)
        self.pub_mode = rospy.Publisher('/cmd_mode', String, queue_size=10)

        print(torch.cuda.is_available())
        print(torch.cuda.get_device_name())

        self.audioMemory = []
        self.generalMode = "undefined" #undefined, find_hum, vel_command, pose_command
        self.soundMode = 'down'  #modes : stop, yes, down, right, undefined
        self.soundDirection = 'infront' #right or left
        self.personInFront = None
        self.poseDirection = None
        self.pitch = 0
        self.roll = 0

        self.posDict = {
            "nose": 0,
            "left_eye": 1,
            "right_eye": 2,
            "left_ear": 3,
            "right_ear": 4,
            "left_shoudler": 5,
            "right_shoudler": 6,
            "left_elbow": 7,
            "right_elbow": 8,
            "left_wrist": 9,
            "right_wrist": 10,
            "left_hip": 11,
            "right_hip": 12,
            "left_knee": 13,
            "right_knee": 14,
            "left_ankle": 15,
            "right_ankle": 16}
        

    def unit_vector(self, vector):
            """ Returns the unit vector of the vector.  """
            return vector / np.linalg.norm(vector)

    def compute_angle(self, shoudler, hip, elbow):
        """ Returns the angle in radiants between of the arm defined by its shoudler, hip and elbow"""
        shoudler = np.asarray([shoudler.x, shoudler.y])
        hip = np.asarray([hip.x, hip.y])
        elbow = np.asarray([elbow.x, elbow.y])
        v1 = self.unit_vector(hip - shoudler)
        v2 = self.unit_vector(elbow - shoudler)
        return np.arccos(np.clip(np.dot(v1, v2),-1.0,1.0))*360/(2*np.pi)

    def compute_arms_angle(self, keypoints):
        """ Returns the angle of left and right arm wrt to the torso (return nan if the arm is not seen) """
        
        pos_dict = {
            "nose": 0,
            "left_eye": 1,
            "right_eye": 2,
            "left_ear": 3,
            "right_ear": 4,
            "left_shoudler": 5,
            "right_shoudler": 6,
            "left_elbow": 7,
            "right_elbow": 8,
            "left_wrist": 9,
            "right_wrist": 10,
            "left_hip": 11,
            "right_hip": 12,
            "left_knee": 13,
            "right_knee": 14,
            "left_ankle": 15,
            "right_ankle": 16}
   

        # left arm
        left_shoulder = keypoints[self.posDict["left_shoudler"]]
        left_hip = keypoints[self.posDict["left_hip"]]
        left_elbow = keypoints[self.posDict["left_elbow"]]

        # right arm
        right_shoulder = keypoints[self.posDict["right_shoudler"]]
        right_hip = keypoints[self.posDict["right_hip"]]
        right_elbow = keypoints[self.posDict["right_elbow"]]

        return(self.compute_angle(left_shoulder, left_hip, left_elbow), self.compute_angle(right_shoulder, right_hip, right_elbow))
    
    def angles_to_command(self, left_angle, right_angle):
        """Returns the command corresponding to arms angle"""
        
        up_angle = 160
        horizontal_angle = 90
        down_angle = 20
        margin = 15

        # backward angle = 160 +/- 10
        if (left_angle > up_angle-margin and left_angle < up_angle+margin) and (right_angle > up_angle-margin and right_angle < up_angle+margin):
            return "backward"
        # forward angle = 90 +/- 10
        if (left_angle > horizontal_angle-margin and left_angle < horizontal_angle+margin) and (right_angle > horizontal_angle-margin and right_angle < horizontal_angle+margin):
            return "forward"
        # left = left: 90 +/- 10 right: 20 +/- 10
        if (left_angle > horizontal_angle-margin and left_angle < horizontal_angle+margin) and (right_angle > down_angle-margin and right_angle < down_angle+margin):
            return "left"
        # right = left: 20 +/- 10 right: 90 +/- 10
        if (left_angle > down_angle-margin and left_angle < down_angle+margin) and (right_angle > horizontal_angle-margin and right_angle < horizontal_angle+margin):
            return "right"
        else:
            return "stop"
                
    def frame_to_vel_command(self, keypoints):
        """Returns the command related to the body pose of user"""
        try:
            left_angle, right_angle = self.compute_arms_angle(keypoints)
            command = self.angles_to_command(left_angle, right_angle)
            print(command)
        
        except:
            command = "stop"
            print("nobody in frame")

        return command
    
    def person_in_front(self, keypoints): 
        '''Returns true if human in front of camera''' #guiguiii
        left_shoulder = keypoints[self.posDict["left_shoudler"]]
        right_shoulder = keypoints[self.posDict["right_shoudler"]]
        mid_shoulders = (left_shoulder.x+right_shoulder.x)/2
        mid_image = 320 # hard coded
        thresh = 35
    #    print(mid_shoulders)
    #    print(mid_image)
        if abs(mid_shoulders-mid_image) < thresh:
            if(not(self.personInFront)):
                print("found human in front!!")
            return True
        else:
    #        print("PERSON NOT in front XXX")
            return False


    def callbackFrame(self, data):
  #      print("frame callback")
        keypoints = data.keypoints
        pitch = data.pitch
        roll = data.roll 
        self.personInFront =  self.person_in_front(keypoints)
        if(self.generalMode=="vel_command"):
            self.poseDirection =  self.frame_to_vel_command(keypoints)
        elif(self.generalMode=="pose_command"):
            self.roll = roll
            self.pitch = pitch

        

    def callbackAudio(self, data):
        print("IN AUDIOCALLBACK")
        data = str(data)[7:-1]  #remove the "data: " and the "'" at the end
        data = data.split("#")
        self.soundMode = data[0]
        print("heard soundmode: "+self.soundMode)

        if self.soundMode == "stop":
            print("changement de mode en mode find human (stop entendu)")
            self.soundDirection = data[1]
            self.audioMemory.append(self.soundDirection) # add new sound direction to memory
            self.audioMemory = self.audioMemory[:-10]
            self.generalMode = "find_human"
        if self.soundMode == "yes":
            print("changement de mode en mode vel command (yes entendu)")
            self.generalMode = "vel_command"
        if self.soundMode == "right":
            print("changemende de mode en mode command_pose (right entendu)")
            self.generalMode = "pose_command"
        if self.soundMode == "down":
            print("changemende de mode en mode undefined (down entendu)")
            self.generalMode = "undefined"
        else:
            pass

        print("actual generalmode: "+self.generalMode)


        
    def info_to_vel(self):
        vel_msg = Twist()
        pose_msg = Twist()
        
      #  print("actual generalmode: "+self.generalMode)

        #if the previous mode was to find human, 
        #if(self.generalMode=="find_human"):
        #    print("waiting to receive audio order")
        #    rospy.wait_for_message("/audio_info_topic", String, timeout=None)

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
                self.generalMode = 'undefined'

            while(not(self.personInFront)):
                if(self.soundDirection == 'right'):
                    print("turning RIGHT")
                    #vitesse de rotation negative
                    vel_msg.angular.z = -0.2

                elif(self.soundDirection=='left'):
                    print("turning LEFT")
                    #vitesse de rotation positive
                    vel_msg.angular.z = 0.2

                elif counter == 5: 
                    print("movement finished BUT NO HUMAN FOUND")
                    vel_msg.angular.z = 0
                else:
                    print('human behind')
                    vel_msg.angular.z = 0.2
                
                print(time.perf_counter() - timer)
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


        if(self.generalMode == 'vel_command'): 

            # Process the image to determine direction

            if self.poseDirection == 'forward':
                print('move forward')
                vel_msg.linear.x = 0.5
                vel_msg.linear.y = 0


            elif self.poseDirection == 'backward':
                print('move backward')
                vel_msg.linear.x = -0.5
                vel_msg.linear.y = 0

            elif self.poseDirection == 'left':
                print('move left')
                vel_msg.linear.y = 0.5
                vel_msg.linear.x = 0

            elif self.poseDirection == 'right':
                print('move right')
                vel_msg.linear.y = -0.5
                vel_msg.linear.x = 0

            elif self.poseDirection == 'stop':
                print('move stop')
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
            else:
                print('unknown poseDirection')
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                
            self.pub_mode.publish(self.generalMode)
            self.pub_vel.publish(vel_msg)

        elif(self.generalMode == 'pose_command'):
            #print('in pose command')
            if(abs(self.roll)<ROLL_MAX and abs(self.pitch)<PITCH_MAX):        
                pose_msg.angular.x = deg2rad(self.roll)
                pose_msg.angular.y = deg2rad(self.pitch)
            elif(abs(self.roll)<ROLL_MAX):
                pose_msg.angular.x = deg2rad(self.roll)
                pose_msg.angular.y = deg2rad(PITCH_MAX)*np.sign(self.pitch)
            elif(abs(self.pitch)<PITCH_MAX):
                pose_msg.angular.x = deg2rad(ROLL_MAX)*np.sign(self.roll)
                pose_msg.angular.y = deg2rad(self.pitch)
            else: 
                pose_msg.angular.x = deg2rad(ROLL_MAX)*np.sign(self.roll)
                pose_msg.angular.y = deg2rad(PITCH_MAX)*np.sign(self.pitch)
            
            print(f"Roll: {pose_msg.angular.x}, Pitch: {pose_msg.angular.y}")

            self.pub_mode.publish(self.generalMode)
            self.pub_pose.publish(pose_msg)


        elif(self.generalMode == 'undefined'):
            #print("ROBOT AT REST")
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0 
            vel_msg.angular.z = 0

        # Publish the velocity command
        self.pub_vel.publish(vel_msg)
        self.pub_mode.publish(self.generalMode)


    
if __name__ == '__main__':

    try:
        ip = ImageProcessor()
        while not rospy.is_shutdown():
            ip.info_to_vel()
    except rospy.ROSInterruptException:
        pass

