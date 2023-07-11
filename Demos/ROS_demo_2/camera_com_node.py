#!/data/grebici/leg_env/bin/python

###!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import openpifpaf
from cv_bridge import CvBridge
import numpy as np
import PIL
import torch


class ImageProcessor:
    def __init__(self):
        predictor = openpifpaf.Predictor(checkpoint='shufflenetv2k16')
        rospy.init_node('image_processor')
        rospy.Subscriber('/camera/color/image_raw', Image, self.callback, callback_args=predictor)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        print(torch.cuda.is_available())
        print(torch.cuda.get_device_name())

    def unit_vector(self, vector):
            """ Returns the unit vector of the vector.  """
            return vector / np.linalg.norm(vector)

    def compute_angle(self, shoudler, hip, elbow):
        """ Returns the angle in radiants between of the arm defined by its shoudler, hip and elbow"""
        v1 = self.unit_vector(hip - shoudler)
        v2 = self.unit_vector(elbow - shoudler)
        return np.arccos(np.clip(np.dot(v1, v2),-1.0,1.0))*360/(2*np.pi)

    def compute_arms_angle(self, prediction):
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
        
        # we suppose only on person on the image (index = 0)
        keypoints = prediction[0].data

        # left arm
        left_shoulder = keypoints[pos_dict["left_shoudler"],0:2]
        left_hip = keypoints[pos_dict["left_hip"],0:2]
        left_elbow = keypoints[pos_dict["left_elbow"],0:2]

        # right arm
        right_shoulder = keypoints[pos_dict["right_shoudler"],0:2]
        right_hip = keypoints[pos_dict["right_hip"],0:2]
        right_elbow = keypoints[pos_dict["right_elbow"],0:2]

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
                
    def callback(self, data, args):
        predictor = args
        # Convert the image to OpenCV format
        print(type(data))
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        cv_image = cv2.resize(cv_image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA) 

        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        pil_img = PIL.Image.fromarray(img)

        with torch.no_grad():
            predictions, _, _ = predictor.pil_image(pil_img)

        try:
            left_angle, right_angle = self.compute_arms_angle(predictions)
            command = self.angles_to_command(left_angle, right_angle)
            print(command)
        
        except:
            command = "stop"
            print("nobody in frame")


        print(cv_image.shape)
        # Process the image to determine direction
        # (replace this with your own image processing code)
        direction = command
        print('---------------- image recue ------------------')

        # Create a Twist message with the appropriate velocity command
        vel_msg = Twist()
        if direction == 'forward':
            vel_msg.linear.x = 0.5
        elif direction == 'backward':
            vel_msg.linear.x = -0.5
        elif direction == 'left':
            vel_msg.linear.y = 0.5
        elif direction == 'right':
            vel_msg.linear.y = -0.5
        if direction == 'stop':
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0


        # Publish the velocity command
        self.pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        ip = ImageProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

