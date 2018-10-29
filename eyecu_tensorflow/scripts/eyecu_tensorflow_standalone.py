#!/usr/bin/env python


################################################################################
#  Standard python libraries
import numpy as np
import os
import six.moves.urllib as urllib
import sys
import cv2
from imutils.video import WebcamVideoStream

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image

import yaml

# Tensorflow
import tensorflow as tf

# ROS libraries
import rospy

# Custom messages
from eyecu_msgs.msg import DistanceCamera

# Path to this package
import rospkg
rospack = rospkg.RosPack()
sys.path.insert(0, rospack.get_path('eyecu_tensorflow'))

# Utils for visualzation and labeling
from utils import label_map_util
from utils import visualization_utils as vis_util


################################################################################
# Tensorflow class
class FaceTensorFlow:
  # Initialization function
  def __init__(self):
    # These variables define the average width and height of human face (in cm)
    self.face_width  = 13.9
    self.face_height = 22.5

    # Get ROS parameters
    self._published_topic = rospy.get_param('published_topic', '/face_distance')
    self._video_device = rospy.get_param('video_device', '/dev/front_camera')

    self._graph_name  = rospy.get_param('graph_name', '/frozen_inference_graph_face.pb')
    self._label_name  = rospy.get_param('label_name', '/face_label_map.pbtxt')
    self._num_classes = rospy.get_param('num_classes', 2)

    self._min_score = rospy.get_param('min_score', 0.5)
    self._display_image = rospy.get_param('display_image', True)

    self._camera_info_path = rospy.get_param('camera_info_path', '/home/yago/catkin_ws/src/eyecu/eyecu/config/logitech_webcam_calibration.yaml')

    # Tensorflow initialization
    self._path_to_ckpt = sys.path[0] + '/exported_graphs' + self._graph_name
    self._path_to_labels = sys.path[0] + '/labels' + self._label_name

    self._detection_graph = tf.Graph()
    self.load_graph()

    self._session = tf.Session(graph=self._detection_graph)

    self._label_map = label_map_util.load_labelmap(self._path_to_labels)
    self._categories = label_map_util.convert_label_map_to_categories(self._label_map,
               max_num_classes=self._num_classes, use_display_name=True)
    self._category_index = label_map_util.create_category_index(self._categories)

    # OpenCV video capture
    self._cap = WebcamVideoStream(src=self._video_device).start()

    # Load values for calibration
    self.load_camera_info()

    # Publisher variable
    self.face_distance = DistanceCamera()

    # Subscribers and publishers
    self._pub = rospy.Publisher(self._published_topic, DistanceCamera, queue_size=1)

  # Graph loading
  def load_graph(self):
    with self._detection_graph.as_default():
      od_graph_def = tf.GraphDef()
      with tf.gfile.GFile(self._path_to_ckpt, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

  def load_camera_info(self):

    try:
      f = open(self._camera_info_path)
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

              print(self._fx)
          except yaml.YAMLError:
              print('Error loading in yaml file. Loading default values')
              self.load_defaults()

    except IOError:
      print('Error opening file. Loading default values')
      self.load_defaults()

  def load_defaults(self):

    self._fx = 507.5024270566367
    self._cx = 322.7029200800868
    self._fy = 507.2559728776117
    self._cy = 239.1426526245542
    self._width  = 640
    self.center_x = self._width/2
    self._height = 480
    self.center_y = self._height/2
    print('Defaults loaded correctly')

  # Image subscriber definition
  def detect_faces(self):

    with self._detection_graph.as_default():
      image_np = self._cap.read()
      # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
      image_np_expanded = np.expand_dims(image_np, axis=0)
      image_tensor = self._detection_graph.get_tensor_by_name('image_tensor:0')
      # Each box represents a part of the image where a particular object was detected.
      boxes = self._detection_graph.get_tensor_by_name('detection_boxes:0')

      # Each score represent how level of confidence for each of the objects.
      # Score is shown on the result image, together with the class label.
      scores = self._detection_graph.get_tensor_by_name('detection_scores:0')
      classes = self._detection_graph.get_tensor_by_name('detection_classes:0')
      num_detections = self._detection_graph.get_tensor_by_name('num_detections:0')

      # Actual detection.
      (boxes, scores, classes, num_detections) = self._session.run(
          [boxes, scores, classes, num_detections],
          feed_dict={image_tensor: image_np_expanded})
      # Visualization of the results of a detection.
      vis_util.visualize_boxes_and_labels_on_image_array(
          image_np,
          np.squeeze(boxes),
          np.squeeze(classes).astype(np.int32),
          np.squeeze(scores),
          self._category_index,
          use_normalized_coordinates=True,
          line_thickness=8)

    if self._display_image:
      cv2.imshow('object_detection', image_np)

    self.publish_face_distance(np.squeeze(boxes), np.squeeze(scores))

  def publish_face_distance(self, boxes, scores):

    max_face = -1
    biggest_face = 0
    # We pick the biggest face we find
    for i in range(boxes.shape[0]):

      if scores[i] > self._min_score:

        current_face = (boxes[i][3] - boxes[i][1]) * (boxes[i][2] - boxes[i][0])

        if current_face > biggest_face:
          max_face = i
          biggest_face = current_face
          

    if max_face >= 0:
      xmin = boxes[max_face][1]*self._width
      xmax = boxes[max_face][3]*self._width
      ymin = boxes[max_face][0]*self._height
      ymax = boxes[max_face][2]*self._height

      center_face_x = (xmax + xmin) / 2
      center_face_y = (ymax + ymin) / 2

      self.face_distance.Z = ((self._fx*self.face_width)**2 + (self._fy*self.face_height)**2)/(((xmax-xmin)*self._fx*self.face_width) + ((ymax-ymin)*self._fy*self.face_height))

      self.face_distance.X = (center_face_x - self.center_x) * self.face_distance.Z/self._fx
      self.face_distance.Y = (center_face_y - self.center_y) * self.face_distance.Z/self._fy

      self._pub.publish(self.face_distance)

  def stop_reading(self):
    self._cap.stop()


################################################################################
# Main function
if __name__ == '__main__':

  cv2.namedWindow("object_detection", cv2.WND_PROP_FULLSCREEN)
  cv2.setWindowProperty("object_detection",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)

  tensor = FaceTensorFlow()
  rospy.init_node('face_tracking_tensorflow', anonymous=True)

  while True:
    tensor.detect_faces()

    if cv2.waitKey(25) & 0xFF == ord('q'):
      # out.release()
      tensor.stop_reading()
      cv2.destroyAllWindows()
      break
