#!/usr/bin/env python

from emojiObject import load_emoji, emojiObject
import cv2

if __name__ == "__main__":
    # First test
    path_to_images = "/home/yago/workspaces/catkin_ws/src/eyecu/eyecu_emotion/images"

    # This test should print the dictionary
    try:
        emoji_dict = load_emoji(path_to_images)
        print(emoji_dict['angry'])

        cv2.namedWindow('window_frame', cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty('window_frame', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        cv2.imshow('window_frame', emoji_dict['disgust']._img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    except IOError:
        print("Could not load emoji images")
        exit

    # Second test
    path_to_images = ""

    # This test should fail
    try:
        emoji_dict = load_emoji(path_to_images)
        print(emoji_dict)
    except IOError:
        print("Could not load emoji images")
        exit
    
