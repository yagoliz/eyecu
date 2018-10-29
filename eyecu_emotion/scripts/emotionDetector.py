#!/usr/bin/env python

# Main libraries
import sys
import yaml

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
        # Initialize camera and get its info
        self._camera = rospy.get_param("~/cam_device", "/dev/video0")
        self._camera_calibration_path = rospy.get_param("~/camera_info_path", "/home/yago/catkin_ws/src/eyecu/eyecu/config/logitech_webcam_calibration.yaml")
        self._load_camera_info()

        # Get files for face detection
        self._detection_model_path = rospy.get_param("~/detection_model_path", "/trained_models/detection_models/")
        self._detection_model_file = rospy.get_param("~/detection_model_file", "haarcascade_frontalface_default.xml")
        self._detection_model = sys.path[0] + self._detection_model_path + self._detection_model_file
        self._load_face_detection()

        # Get files for emotion detection
        self._emotion_model_path = rospy.get_param("~/emotion_model_path", "/trained_models/")
        self._emotion_model_file = rospy.get_param("~/emotion_model_file", "fer2013_mini_XCEPTION.119-0.65.hdf5")
        self._emotion_model = sys.path[0] + self._emotion_model_path + self._emotion_model_file
        self._load_emotion_detection()

        # Hyper-parameters for bounding boxes shape
        self._frame_window = 10
        self._emotion_offsets = (20, 40)

        # Publisher topics
        self._face_distance_topic = rospy.get_param("~/face_distance_topic", "/face_distance")
        self._emotion_topic = rospy.get_param("~/emotion_topic", "/emotion_status")

        # Initialize publishers
        self._face_distance_publisher = rospy.Publisher(self._face_distance_topic, DistanceCamera, queue_size=1)
        self._emotion_publisher = rospy.Publisher(self._emotion_topic, Int8, queue_size=1)

        # Start OpenCV video capture
        self._video_capture = WebcamVideoStream(src=self._camera).start()

    # Helper functions to load the detectors
    def _load_face_detection(self):
        self._face_detection = load_detection_model(self._detection_model)

    def _load_emotion_detection(self):
        self._emotion_classifier = load_model(self._emotion_model, compile=False)

    # Load camera information
    def _load_camera_info(self):

        try:
            f = open(self._camera_calibration_path)
            with f as stream:
                try:
                    data = yaml.load(stream)
                    matrix = data['camera_matrix']['data']
                    self._fx = matrix[0]
                    self._cx = matrix[2]
                    self._fy = matrix[4]
                    self._cy = matrix[5]
                    self._width = data['image_width']
                    self.center_x = self._width/2
                    self._height  = data['image_height']
                    self.center_y = self._height/2

                except yaml.YAMLError:
                    print('Error loading yaml file. Loading defaults')
                    self._load_defaults()

        except IOError:
            print('Error opening file. Loading defaults')
            self._load_defaults()

    # This function loads the defaults of a Dell Precision Webcam
    # Do not use if you have another camera
    def _load_defaults(self):
        self._fx = 507.5024270566367
        self._cx = 322.7029200800868
        self._fy = 507.2559728776117
        self._cy = 239.1426526245542
        self._width  = 640
        self.center_x = self._width/2
        self._height = 480
        self.center_y = self._height/2
        print('Defaults loaded correctly')


# Main function definition
if __name__ == "__main__":
    pass