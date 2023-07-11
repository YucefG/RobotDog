###!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import sys
#sys.path.append('/data/grebici/catkin_WBC/src/my_pkgs/custom_msg')
from custom_msg.msg import Frame_info
from geometry_msgs.msg import Point
import numpy as np
import pyrealsense2 as rs
import openpifpaf
import PIL
import math as m
import json

DIM3 = True  #if true, send 3d poses
PLANE_DIST = 0.3 #dist btwn hip and plane in x direction

def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix

def cart2sph(x,y,z):
    XsqPlusYsq = x**2 + y**2
    r = m.sqrt(XsqPlusYsq + z**2)               # r
    elev = m.atan2(z,m.sqrt(XsqPlusYsq))     # theta
    az = m.atan2(y,x)                           # phi
    return r, elev, az

def cart2pitchRoll(x,y,z):
    pitch = m.atan2(x,z)
    roll = m.atan2(y,z)
    return roll,pitch

def rad2deg(rad):
    return rad*180/m.pi


if __name__ == '__main__':
    ### CONSTANTS ###
    DETECTION = True
    BASE_ORIENTATION = [0, 0, -1]
    predictor = openpifpaf.Predictor(checkpoint='shufflenetv2k16') #could use mobilenetv3small for betterperformances
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
    ### END CONSTANTS ###
    
    rospy.init_node('keypoints_detector_node')

    # Create a publisher for the keypoints array
    keypoints_pub = rospy.Publisher('video_info_topic', Frame_info, queue_size=10)

    # Configure the RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.gyro)
    config.enable_stream(rs.stream.accel)
    
    # Initialize the RealSense pipeline
    cfg = pipeline.start(config)
    colorizer = rs.colorizer()

    profile = cfg.get_stream(rs.stream.depth) # Fetch stream profile for depth stream
    intr = profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
    try: 
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames() # depth, rgb, accel, gyro
            print("---frame acquired---")
            accel_frame = frames[2]
            gyro_frame = frames[3]
        
            if gyro_frame:
                gyro_data = gyro_frame.as_motion_frame().get_motion_data()

            if accel_frame:
                accel_data = accel_frame.as_motion_frame().get_motion_data()
                orientation = [accel_data.x, accel_data.y, accel_data.z]
                orientation[1], orientation[2] = orientation[2], orientation[1] #swap y and z
                rotation = rotation_matrix_from_vectors(orientation, BASE_ORIENTATION) # rotation required to be aligned with gravity

            # Create alignment primitive with color as its target stream:
            align = rs.align(rs.stream.color)
            frameset = align.process(frames)

            # get aligned frames
            depth_frame = frameset.get_depth_frame()
            color_frame = frameset.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # ==== OPENPIFPAF =====
            if DETECTION:
                # img = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB)
                pil_img = PIL.Image.fromarray(color_image)
                frame = color_image

                predictions, _, _ = predictor.pil_image(pil_img)
                

                try:
                    message = Frame_info()

                    # keypoints
                    keypoints = predictions[0].data # 0 because we take the first person (should be only 1 in frame)

                    # right arm
                    right_elbow = keypoints[pos_dict["right_elbow"],0:2]
                    right_wrist = keypoints[pos_dict["right_wrist"],0:2]

                    # shoulders
                    right_shoulder = keypoints[pos_dict["right_shoudler"],0:2]
                    left_shoulder = keypoints[pos_dict["left_shoudler"],0:2]
                    middle_shoulders = (right_shoulder+left_shoulder)/2


                    # get 3d coordinates of the right elbow and wrist
                    right_elbow_3d = rs.rs2_deproject_pixel_to_point(intr, right_elbow, depth_frame.get_distance(int(right_elbow[0]), int(right_elbow[1])))
                    right_wrist_3d = rs.rs2_deproject_pixel_to_point(intr, right_wrist, depth_frame.get_distance(int(right_wrist[0]), int(right_wrist[1])))
                    
                    if(DIM3):
                        # get 3d coordinates of middle point of shoulders
                        middle_shoulders_3d = rs.rs2_deproject_pixel_to_point(intr, middle_shoulders, depth_frame.get_distance(int(middle_shoulders[0]), int(middle_shoulders[1])))

                        # compute vector from elbow to wrist
                        right_wrist_3d = np.array(right_wrist_3d)
                        middle_shoulders_3d = np.array(middle_shoulders_3d)
                        print(f"middle shoulders:{middle_shoulders_3d}")
                        print(f"right wrist:{right_wrist_3d}")

                        ee = right_wrist_3d - middle_shoulders_3d 
                        message.ee.x = ee[0]
                        message.ee.y = ee[1]
                        message.ee.z = ee[2]
                    else:
                        message.ee.x = PLANE_DIST
                        message.ee.y = -(right_wrist[0] - middle_shoulders[0])*0.001
                        message.ee.z = -(right_wrist[1] - middle_shoulders[1])*0.001


                    right_forearm_vector = np.array(right_wrist_3d) - np.array(right_elbow_3d)
                    
                    # coorect vector orientation
                    right_forearm_vector[[1,2]] = right_forearm_vector[[2,1]] # swap y and z
                    corrected_forearm_vector = rotation.dot(right_forearm_vector) # rotate forearm vector to align with gravity
                    corrected_forearm_vector[2] = -corrected_forearm_vector[2] # invert z to have positive elevation when arm is raised

                    # convert to spherical coordinates
                    _, elevation, azimuth = cart2sph(*corrected_forearm_vector)
                    elevation = rad2deg(elevation)
                    azimuth = rad2deg(azimuth)

                    pitch ,roll = cart2pitchRoll(*corrected_forearm_vector)
                    pitch = rad2deg(pitch)
                    roll = rad2deg(roll)
                    

                    for i, keypoint in enumerate(keypoints):
                            point = Point()
                            point.x = keypoint[0]
                            point.y = keypoint[1]
                #            message.keypoints[i].x = keypoint[0]
                #            message.keypoints[i].y = keypoint[1]
                            message.keypoints.append(point)
                        
                    message.pitch = pitch
                    message.roll = roll

                except:
                    print("issue with keypoint detection (missing eypoints/no person detected)")
                    message = Frame_info()
                    # create a message filled with zeros when no detection
                    for i in range(17):
                        point = Point()
                        point.x = 0
                        point.y = 0
                        message.keypoints.append(point)
                                            
                    message.pitch = 0
                    message.roll = 0

                  #  message.ee.x = PLANE_DIST
                  #  message.ee.y = 0.1
                  #  message.ee.z = 0

                    pass
            # ==== END OPENPIFPAF =====
            
            keypoints_pub.publish(message)
    except KeyboardInterrupt:
        pipeline.stop()
        print("STOPPEDDDDDDD")
    

    
    
   
