#!/usr/bin/env python

import yaml

try:
  f = open('/hme/yago/catkin_ws/src/eyecu/face_tracker_pkg/config/logitech_webcam_calibration.yaml', 'r')
  print('HEY')
except IOError:
  print ('LOL')


with f as stream:
    try:
        data = yaml.load(stream)
    except yaml.YAMLError as exc:
        print('LOL')
matrix = data['camera_matrix']['data']

print matrix[0]
