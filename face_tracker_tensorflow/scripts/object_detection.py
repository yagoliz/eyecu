#!/usr/bin/env python

import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image

import cv2

from imutils.video import WebcamVideoStream

cap = WebcamVideoStream(src=1).start()
# cap = cv2.VideoCapture(0)
cv2.namedWindow("object_detection", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("object_detection",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)

# This is needed since the notebook is stored in the object_detection folder.
sys.path.insert(0, "/home/yago/Documents/test")
from utils import label_map_util
from utils import visualization_utils as vis_util

# Video recording files
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# out = cv2.VideoWriter('2018_05_13_face_recognition.mp4',fourcc, 20.0, (640,480))

# ## Object detection imports
# Here are the imports from the object detection module.




# # Model preparation

# ## Variables
#
# Any model exported using the `export_inference_graph.py` tool can be loaded here simply by changing `PATH_TO_CKPT` to point to a new .pb file.
#
# By default we use an "SSD with Mobilenet" model here. See the [detection model zoo](https://github.com/tensorflow/models/blob/master/object_detection/g3doc/detection_model_zoo.md) for a list of other models that can be run out-of-the-box with varying speeds and accuracies.

# What model to download.
MODEL_NAME = 'exported_graphs'

# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph_face.pb'

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = os.path.join('labels', 'face_label_map.pbtxt')

NUM_CLASSES = 2

# ## Load a (frozen) Tensorflow model into memory.

detection_graph = tf.Graph()
with detection_graph.as_default():
  od_graph_def = tf.GraphDef()
  with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(od_graph_def, name='')


# ## Loading label map
# Label maps map indices to category names, so that when our convolution network predicts `5`, we know that this corresponds to `airplane`.  Here we use internal utility functions, but anything that returns a dictionary mapping integers to appropriate string labels would be fine

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)


# # Detection

with detection_graph.as_default():
  with tf.Session(graph=detection_graph) as sess:
    while True:
      image_np = cap.read()
      # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
      image_np_expanded = np.expand_dims(image_np, axis=0)
      image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
      # Each box represents a part of the image where a particular object was detected.
      boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
      # Each score represent how level of confidence for each of the objects.
      # Score is shown on the result image, together with the class label.
      scores = detection_graph.get_tensor_by_name('detection_scores:0')
      classes = detection_graph.get_tensor_by_name('detection_classes:0')
      num_detections = detection_graph.get_tensor_by_name('num_detections:0')
      # Actual detection.
      (boxes, scores, classes, num_detections) = sess.run(
          [boxes, scores, classes, num_detections],
          feed_dict={image_tensor: image_np_expanded})
      # Visualization of the results of a detection.
      vis_util.visualize_boxes_and_labels_on_image_array(
          image_np,
          np.squeeze(boxes),
          np.squeeze(classes).astype(np.int32),
          np.squeeze(scores),
          category_index,
          use_normalized_coordinates=True,
          line_thickness=8)

      # cv2.imshow('object_detection', cv2.resize(image_np, (19,600)))
      cv2.imshow('object_detection', image_np)
      # out.write(image_np)

      if cv2.waitKey(25) & 0xFF == ord('q'):
        # out.release()
        cv2.destroyAllWindows()
        break
