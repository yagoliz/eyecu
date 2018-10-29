#!/usr/bin/env python

from emotionDetector import load_emoji, emojiObject

if __name__ == "__main__":
    # First test
    path_to_images = "/home/yago/workspaces/catkin_ws/src/eyecu/eyecu_emotion/images"

    # This test should print the dictionary
    try:
        emoji_dict = load_emoji(path_to_images)
        print(emoji_dict)
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
    
