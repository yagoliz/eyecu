#!/usr/bin/env python

# Main libraries
import sys

# Emoji object
from emojiObject import *

# Installed libraries
from keras.models import load_model
import numpy as np
from statistics import mode
from imutils.video import WebcamVideoStream

# ROS
import rospy
from std_msgs.msg import Int8
# Get path to this pkg
import rospkg
rospack = rospkg.RosPack()
sys.path.insert(0, rospack.get_path('eyecu_emotion'))

# Custom ROS messages
from eyecu.msg import DistanceCamera

# Utilities
from utils.datasets import get_labels
from utils.inference import detect_faces
from utils.inference import draw_text
from utils.inference import draw_bounding_box
from utils.inference import apply_offsets
from utils.inference import load_detection_model
from utils.preprocessor import preprocess_input

# Emotion detector class
class emotionDetector(object):
    # Init function takes a dictionary as an argument
    def __init__(self, emoji_dictionary):
        # Values for average face dimensions
        self._face_width = 13.9 # in cm
        self._face_height = 22.5

        # Emoji dictionary
        self._emoji_dictionary = emoji_dictionary

        # ROS parameters
        self._camera = rospy.get_param("~/cam_device", "/dev/video0")
        self._camera_calibration_path = rospy.get_param("~/camera_info_path", "/home/yago/catkin_ws/src/eyecu/eyecu/config/logitech_webcam_calibration.yaml")

        self._emotion_topic = rospy.get_param("~/emotion_topic", "/emotion_status")
        self._face_distance_topic = rospy.get_param("~/face_distance_topic", "/face_distance")

        self._detection_model_path = rospy.get_param("detection_model_path")

        # Initialize publishers
        self._emotion_publisher = rospy.Publisher("")