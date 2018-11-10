#!/usr/bin/env python

# Installed libraries
import cv2
import numpy as np

# Create supporting classes
class emojiObject(object):
    def __init__(self):
        # Variable to contain the emoji image:
        self._img = []
        # Next variables are used to change the desired face for an emoji
        # Variable to contain the mask
        self._mask = []
        # Variable to contain the inverse mask
        self._mask_inverse = []

    # This function will set the data to the object
    def set_data(self, data):
        self._img = data[0]
        self._mask = data[1]
        self._mask_inverse = data[2]

    # Get data in tuple form
    def get_data(self):
        return (self._img, self._mask, self._mask_inverse)

# Emoji loader function
def load_emoji(path_to_images):
    '''
    This function returns an emoji list dictionary given the path to the
    images as a string
    '''
    # Create empty dictionary
    emoji_dictionary = {}

    # List all emotions names
    emoji_list = ['angry', 'disgust','sad', 'neutral', 'happy' ]
    for emoji in emoji_list:
        # Create emoji object
        emoji_object = emojiObject()

        # Load image
        filename = path_to_images + "/" + emoji + ".png"
        emoji_image = cv2.imread(filename, -1)
        if not isinstance(emoji_image, np.ndarray):
            raise IOError("Error loading image")

        # Create the masks for the emoji
        emoji_mask = emoji_image[:, :, 3]
        emoji_mask_inverted = cv2.bitwise_not(emoji_mask)

        # Save only the first 3 dimensions (to compare against video capture frame)
        emoji_image = emoji_image[:, :, 0:3]

        # Set the values to the object
        emoji_object.set_data((emoji_image, emoji_mask, emoji_mask_inverted))
        emoji_dictionary[emoji] = emoji_object

    # Return the completed dictionary
    return emoji_dictionary