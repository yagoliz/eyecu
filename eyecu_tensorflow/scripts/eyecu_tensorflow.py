#!/usr/bin/env python


################################################################################
#  Standard python libraries
import numpy as np
import os
import six.moves.urllib as urllib
import sys
import cv2
import tensorflow as tf

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image

# ROS libraries
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError

# Custom messages
from eyecu_msgs.msg import boundingBox
from eyecu_msgs.msg import boundingBoxArray

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
    # Get ROS parameters
    self._camera_topic = rospy.get_param('camera_topic','/front_camera'  )
    self._image_topic  = rospy.get_param('image_topic' ,'/image_raw')
    self._camera_info_topic = rospy.get_param('camera_info_topic', '/camera_info')
    self._published_topic = rospy.get_param('published_topic', '/bounding_boxes')

    self._graph_name  = rospy.get_param('graph_name', '/frozen_inference_graph_face.pb')
    self._label_name  = rospy.get_param('label_name', '/face_label_map.pbtxt')
    self._num_classes = rospy.get_param('num_classes', 2)

    self._display = rospy.get_param('display_result', False)

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

    self._min_score = 0.5

    # OpenCV and bridge variables
    self._cv_bridge = CvBridge()
    # Load default values for calibration
    self._width  = 640
    self._height = 480

    # Subscribers and publishers
    self._image_sub = rospy.Subscriber((self._camera_topic + self._image_topic), Image, self.image_callback, queue_size=1)
    self._camera_info_sub = rospy.Subscriber((self._camera_topic + self._camera_info_topic),
                          CameraInfo, self.camera_info_callback, queue_size=1)
    self._pub = rospy.Publisher(self._published_topic, boundingBoxArray, queue_size=1)

  # Graph loading
  def load_graph(self):
    with self._detection_graph.as_default():
      od_graph_def = tf.GraphDef()
      with tf.gfile.GFile(self._path_to_ckpt, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

  # Image subscriber definition
  def image_callback(self, image_msg):

    try:
      image_np = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv2.waitKey(3)

    with self._detection_graph.as_default():

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

    if self._display:
      cv2.imshow('object_detection', image_np)

    self.publish_bboxes(np.squeeze(boxes), np.squeeze(scores))

  # # Camera info subscriber definition
  def camera_info_callback(self, camera_info_msg):
    self._width  = camera_info_msg.width
    self._height = camera_info_msg.height

  # Publish bounding boxes
  def publish_bboxes(self, boxes, scores):

    bbox = boundingBox()
    bboxes = boundingBoxArray()

    max_face = -1
    # We pick the biggest face we find
    for i in range(boxes.shape[0]):
      if scores[i] > self._min_score:
        max_face = i

    if max_face >= 0:
      xmin = boxes[max_face][1]*self._width
      xmax = boxes[max_face][3]*self._width
      ymin = boxes[max_face][0]*self._height
      ymax = boxes[max_face][2]*self._height

      bbox.name = "face"
      bbox.pointA.x = xmin
      bbox.pointA.y = ymin
      bbox.pointB.x = xmax
      bbox.pointB.y = ymax

      bboxes = [bbox]

      self._pub.publish(bboxes)



################################################################################
# Main function
if __name__ == '__main__':

  # cv2.namedWindow("object_detection", cv2.WND_PROP_FULLSCREEN)
  # cv2.setWindowProperty("object_detection",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)

  tensor = FaceTensorFlow()
  rospy.init_node('face_tracking_tensorflow', anonymous=True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
