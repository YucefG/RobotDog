import io
import numpy as np
import PIL
import torch
import openpifpaf
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from timeit import timeit
from time import time
import os
import cv2
import pyrealsense2 as rs


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


def compute_angle(shoudler, hip, elbow):
    """ Returns the angle in radiants between of the arm defined by its shoudler, hip and elbow"""
    v1 = unit_vector(hip - shoudler)
    v2 = unit_vector(elbow - shoudler)
    return np.arccos(np.clip(np.dot(v1, v2),-1.0,1.0))*360/(2*np.pi)


def compute_arms_angle(prediction):
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
    keypoints = predictions[0].data

    # left arm
    left_shoulder = keypoints[pos_dict["left_shoudler"],0:2]
    left_hip = keypoints[pos_dict["left_hip"],0:2]
    left_elbow = keypoints[pos_dict["left_elbow"],0:2]

    # right arm
    right_shoulder = keypoints[pos_dict["right_shoudler"],0:2]
    right_hip = keypoints[pos_dict["right_hip"],0:2]
    right_elbow = keypoints[pos_dict["right_elbow"],0:2]

    return(compute_angle(left_shoulder, left_hip, left_elbow), compute_angle(right_shoulder, right_hip, right_elbow))


def angles_to_command(left_angle, right_angle):
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
    

RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
YELLOW = (0, 255, 255)

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
    
    

predictor = openpifpaf.Predictor(checkpoint='shufflenetv2k30-wholebody')

pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


# Start streaming
#pipeline.stop()
pipeline.start(config)

debug = False

while True:

    c = cv2.waitKey(1)
    if c == 27: # press escape to quit
        break

    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        continue

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    
    # ==== OPENPIFPAF =====
    # img = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB)
    pil_img = PIL.Image.fromarray(color_image)
    frame = color_image

    predictions, _, _ = predictor.pil_image(pil_img)

    if debug:
        print(f"Angle of the left arm : {left_angle}\nAngle of the right arm : {right_angle}\n")

    try:
        left_angle, right_angle = compute_arms_angle(predictions)

        # writes desired command
        command = angles_to_command(left_angle, right_angle)
        cv2.putText(img = frame, text=f"command: {command}", org = (0,60), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=GREEN,thickness=3)

        #draw arms and hips
        keypoints = predictions[0].data

        # left arm
        left_shoulder = keypoints[pos_dict["left_shoudler"],0:2]
        left_hip = keypoints[pos_dict["left_hip"],0:2]
        left_elbow = keypoints[pos_dict["left_elbow"],0:2]

        # right arm
        right_shoulder = keypoints[pos_dict["right_shoudler"],0:2]
        right_hip = keypoints[pos_dict["right_hip"],0:2]
        right_elbow = keypoints[pos_dict["right_elbow"],0:2]

        cv2.line(frame, tuple(map(int, tuple(left_shoulder))), tuple(map(int, tuple(left_elbow))), RED, 5)
        cv2.line(frame, tuple(map(int, tuple(left_shoulder))), tuple(map(int, tuple(left_hip))), YELLOW, 5)
        cv2.line(frame, tuple(map(int, tuple(right_shoulder))), tuple(map(int, tuple(right_elbow))), RED, 5)
        cv2.line(frame, tuple(map(int, tuple(right_shoulder))), tuple(map(int, tuple(right_hip))), YELLOW, 5)
    except:
        cv2.putText(img = frame, text=f"no detection", org = (0,60), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=GREEN,thickness=2)

    # ==== OPENPIFPAF =====

    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', frame)
    cv2.waitKey(1)

# Stop streaming
pipeline.stop()

# Release resources
cv2.destroyAllWindows()