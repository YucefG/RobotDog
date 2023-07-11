#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from your_keypoints_msgs.msg import KeypointsArray
import numpy as np
import pyrealsense2 as rs
import openpifpaf
import PIL
import math as m

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
    keypoints_pub = rospy.Publisher('keypoints_array', KeypointsArray, queue_size=10)

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
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames() # depth, rgb, accel, gyro

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
                # ==== skeleton drawing ====
                # keypoints
                keypoints = predictions[0].data

                # right arm
                right_elbow = keypoints[pos_dict["right_elbow"],0:2]
                right_wrist = keypoints[pos_dict["right_wrist"],0:2]

                # get 3d coordinates of the right elbow and wrist
                right_elbow_3d = rs.rs2_deproject_pixel_to_point(intr, right_elbow, depth_frame.get_distance(int(right_elbow[0]), int(right_elbow[1])))
                right_wrist_3d = rs.rs2_deproject_pixel_to_point(intr, right_wrist, depth_frame.get_distance(int(right_wrist[0]), int(right_wrist[1])))

                # compute vector from elbow to wrist
                right_forearm_vector = np.array(right_wrist_3d) - np.array(right_elbow_3d)
                
                # coorect vector orientation
                right_forearm_vector[[1,2]] = right_forearm_vector[[2,1]] # swap y and z
                corrected_forearm_vector = rotation.dot(right_forearm_vector) # rotate forearm vector to align with gravity
                corrected_forearm_vector[2] = -corrected_forearm_vector[2] # invert z to have positive elevation when arm is raised

                # convert to spherical coordinates
                _, elevation, azimuth = cart2sph(*corrected_forearm_vector)
                elevation = rad2deg(elevation)
                azimuth = rad2deg(azimuth)

            except:
                print("issue with keypoint detection (missing eypoints/no person detected)")")
                pass
        # ==== END OPENPIFPAF =====
        
        keypoints_pub.publish(keypoints, elevation, azimuth)

    
    
   
