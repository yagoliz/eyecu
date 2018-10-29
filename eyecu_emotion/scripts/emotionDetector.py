#!/usr/bin/env python

# Main libraries
import sys
import yaml
import time

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

        # Get labels and sizes
        self._emotion_labels = get_labels('fer2013')
        self._emotion_target_size = self._emotion_classifier.input_shape[1:3]

        # Hyper-parameters for bounding boxes shape
        self._emotion_window = []
        self._frame_window = 10
        self._emotion_offsets = (20, 40)

        # Publisher topics
        self._face_distance_topic = rospy.get_param("~/face_distance_topic", "/face_distance")
        self._emotion_topic = rospy.get_param("~/emotion_topic", "/emotion_status")

        # Initialize publishers
        self._face_distance_publisher = rospy.Publisher(self._face_distance_topic, DistanceCamera, queue_size=1)
        self._emotion_publisher = rospy.Publisher(self._emotion_topic, Int8, queue_size=1)

        # Display video
        self._display = rospy.get_param("~/display", True)
        self._font = cv2.FONT_HERSHEY_COMPLEX_SMALL
        if self._display:   
            cv2.namedWindow('window_frame', cv2.WND_PROP_FULLSCREEN)
            cv2.setWindowProperty('window_frame', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

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

    def process_image(self):
        # Get the starting time (for FPS calculation)
        time_start = time.time()

        # Get the current frame and convert it to gray and rgb images
        image = self._video_capture.read()
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Detect faces using haarcascades
        faces = detect_faces(self._face_detection, image_gray)

        # Loop over all found faces
        for face_coordinates in faces:
            # Get the coordinates once offsets are applied
            x1, x2, y1, y2 = apply_offsets(face_coordinates, self._emotion_offsets)
            
            # Get the face area and resize
            face_gray = image_gray[y1:y2, x1:x2]
            try:
                face_gray = cv2.resize(face_gray, (self._emotion_target_size))
            except: 
                continue

            # Do some preprocessing before the actual detection
            face_gray = preprocess_input(face_gray, True)
            face_gray = np.expand_dims(face_gray, 0)
            face_gray = np.expand_dims(face_gray, -1)

            # Detect the emotion
            emotion_prediction = self._emotion_classifier.predict(face_gray)
            emotion_probability = np.max(emotion_prediction)
            emotion_label_arg = np.argmax(emotion_prediction)

            # Get the name of the emotion
            emotion_label = self._emotion_labels[emotion_label_arg]
            
            # Check whether emotion is listed within the emojis
            if emotion_label in self._emoji_dictionary:
                with self._emoji_dictionary[emotion_label] as emoji:

                    # Get the dimensions of the face
                    x, y, width, height = face_coordinates

                    # Get the image and the masks
                    (emoji_image, mask, mask_inverse) = emoji.get_data
                    emoji_image = cv2.resize(emoji_image, (width, width), interpolation=cv2.INTER_AREA)
                    mask = cv2.resize(mask, (width, width), interpolation=cv2.INTER_AREA)
                    mask_inverse = cv2.resize(mask_inverse, (width, width), interpolation=cv2.INTER_AREA)

                    # Substract background and foreground
                    roi = image[y:y+width, x:x+width]
                    roi_bg = cv2.bitwise_and(roi, roi, mask=mask_inverse)
                    roi_fg = cv2.bitwise_and(emoji_image, emoji_image, mask=mask)

                    # Create compounded image and substitute it in the colored frame
                    image_destination = cv2.add(roi_bg, roi_fg)
                    image[y:y+height, x:x+width] = image_destination

        # Show image and fps
        if self._display:
            text = "FPS: " + str(1/(time.time() - time_start))
            cv2.putText(image, text, (50, 50), self._font, 2, (250, 128, 114), 1, cv2.LINE_8)

            cv2.imshow('window_frame', image)

        

# Main function definition
if __name__ == "__main__":
    # Initialize node
    rospy.init_node("emotion_detection")

    # Start the emoji dictionary
    path_to_images = rospy.get_param("~/path_to_images", "/home/yago/workspaces/catkin_ws/src/eyecu/eyecu_emotion/images")
    emoji_dictionary = load_emoji(path_to_images)

    # Create the emotion detector object
    emotion_detector = emotionDetector(emoji_dictionary)

    # Create the shutdownhook
    stop = False
    def shutdownhook():
        stop = True

    rospy.on_shutdown(shutdownhook)

    # Start 
    while not stop:
        emotion_detector.process_image()

        # If 'q' key is pressed the loop is broken too
        if cv2.waitKey(33) & 0xFF == ord('q'):
            break

    # Destroy al OpenCV windows
    cv2.destroyAllWindows()