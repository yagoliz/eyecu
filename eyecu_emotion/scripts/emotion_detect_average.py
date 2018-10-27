#!/usr/bin/env python
# for ROS

import os

import rospy
from std_msgs.msg import Int8
import rospkg

import time

from statistics import mode

import cv2
from keras.models import load_model
import numpy as np

from utils.datasets import get_labels
from utils.inference import detect_faces
from utils.inference import draw_text
from utils.inference import draw_bounding_box
from utils.inference import apply_offsets
from utils.inference import load_detection_model
from utils.preprocessor import preprocess_input

from imutils.video import VideoStream

# Class and function definitions
class EmoList:
    def __init__(self):
        self.img =[]
        self.mask = []
        self.mask_inv = []

def load_emo(path_images):
    emo_list = []
    for i in range(0, 5):
        #testing emoji overlay
        emo_obj = EmoList()
        if i == 1:
            file_emo = "angry"
        elif i == 0:
            file_emo = "disgust"
        elif i == 4:
            file_emo = "happy"
        elif i == 3:
            file_emo = "meh"
        elif i == 2:
            file_emo = "tt"
        for j in range(1, 5):

            file_name = path_images + "/images/" + file_emo + str(j) + ".png"
            print file_name
            emo = []
            emo = cv2.imread(file_name, -1)
            # print emo.dtype
            # Create the mask for the emoji
            orig_mask = emo[:, :, 3]
            # Create the inverted mask for the emo
            orig_mask_inv = cv2.bitwise_not(orig_mask)
            # Convert emo image to BGR
            # and save the original image size (used later when re-sizing the image)
            emo = emo[:, :, 0:3]
            origHeight, origWidth = emo.shape[:2]
            emo_obj.img.append(emo)
            emo_obj.mask.append(orig_mask)
            emo_obj.mask_inv.append(orig_mask_inv)
        emo_list.append(emo_obj)

    return emo_list


if __name__ == '__main__':

  # Create emotion dictionary
  emo_dict = {'disgust':0, 'angry':1, 'sad':2, 'neutral':3, 'happy':4}
  keys = emo_dict.keys()

  # Initialize ROS variables
  rospy.init_node('emotion_node', anonymous=True)
  publisher = rospy.Publisher('/emotion_status', Int8, queue_size=1)
  publisher_object = Int8()

  # ROS parameters
  cam_device = rospy.get_param('~/cam_device', '/dev/video0')

  # Set paths to images and keras models
  rospack = rospkg.RosPack()
  path = rospack.get_path('emotion_detection')
  path_images = path[0:len(path)-18]

  # Parameters for loading data and images
  detection_model_path = path + '/trained_models/detection_models/haarcascade_frontalface_default.xml'
  emotion_model_path = path + '/trained_models/emotion_models/fer2013_mini_XCEPTION.102-0.66.hdf5'
  emotion_labels = get_labels('fer2013')

  # Load emoji
  emoji_list = load_emo(path_images)
  frame_num = 0
  last_time = int(round(time.time() * 1000))

  # Hyper-parameters for bounding boxes shape
  frame_window = 10
  emotion_offsets = (20, 40)

  # Loading models
  face_detection = load_detection_model(detection_model_path)
  emotion_classifier = load_model(emotion_model_path, compile=False)

  # Getting input model shapes for inference
  emotion_target_size = emotion_classifier.input_shape[1:3]

  # Starting lists for calculating modes
  emotion_window = []

  # Starting video streaming
  cv2.namedWindow('window_frame', cv2.WND_PROP_FULLSCREEN)
  cv2.setWindowProperty('window_frame', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
  # video_capture = cv2.VideoCapture('/dev/driver_cam')
  video_capture = VideoStream(src=cam_device).start()
  # video_capture.set(3,640);
  # video_capture.set(4,480);
  # video_capture_front = cv2.VideoCapture('/dev/front_cam')
  # video_capture_front.set(3,640);
  # video_capture_front.set(4,480);
  while not rospy.is_shutdown():

    emo_ave = -1

    if int(round(time.time() * 1000)) - last_time > 500:
        last_time = int(round(time.time() * 1000))
        frame_num = frame_num + 1
        if frame_num > 3:
            frame_num = 0

    # front_image = video_capture_front.read()[1]
    front_image = video_capture.read()
    # bgr_image = video_capture.read()[1]
    bgr_image = front_image
    gray_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
    rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)

    faces = detect_faces(face_detection, gray_image)
    if len(faces)>0:
      emo_ave = 0
      for face_coordinates in faces:


          x1, x2, y1, y2 = apply_offsets(face_coordinates, emotion_offsets)
          gray_face = gray_image[y1:y2, x1:x2]
          try:
              gray_face = cv2.resize(gray_face, (emotion_target_size))
          except:
              continue

          gray_face = preprocess_input(gray_face, True)
          gray_face = np.expand_dims(gray_face, 0)
          gray_face = np.expand_dims(gray_face, -1)

          emotion_prediction = emotion_classifier.predict(gray_face)
          emotion_probability = np.max(emotion_prediction)
          emotion_label_arg = np.argmax(emotion_prediction)

          emotion_text = emotion_labels[emotion_label_arg]
          emotion_window.append(emotion_text)

          if len(emotion_window) > frame_window:
              emotion_window.pop(0)
          try:
              emotion_mode = mode(emotion_window)
          except:
              continue

          if emotion_text in emo_dict:
            emo_id = emo_dict[emotion_text]
            x, y, width, height = face_coordinates
            emoji = cv2.resize(emoji_list[emo_id].img[frame_num],(width,width),interpolation=cv2.INTER_AREA)
            mask = cv2.resize(emoji_list[emo_id].mask[frame_num],(width,width),interpolation=cv2.INTER_AREA)
            mask_inv=cv2.resize(emoji_list[emo_id].mask_inv[frame_num],(width,width),interpolation=cv2.INTER_AREA)
            roi = front_image[y:y+width, x:x+width]
            roi_bg = cv2.bitwise_and(roi, roi, mask=mask_inv)
            roi_fg = cv2.bitwise_and(emoji, emoji, mask=mask)
            dst = cv2.add(roi_bg, roi_fg)
            front_image[y:y+height, x:x+width] = dst
            # bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            emo_ave += emo_id

      emo_ave = int(emo_ave/len(faces))

      for k in keys:
        if emo_dict[k] == emo_ave:
          text = 'Overall feeling: ' + str(emo_ave)
          cv2.putText(front_image, text, (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 255), 2, cv2.LINE_8)

    else:
      text = 'Overall feeling: ' + 'No face detected'
      cv2.putText(front_image, text, (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 255), 2, cv2.LINE_8)

    publisher_object.data = emo_ave
    publisher.publish(publisher_object)

    cv2.imshow('window_frame', front_image)
    if cv2.waitKey(33) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
