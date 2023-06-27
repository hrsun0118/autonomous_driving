#!/usr/bin/env python
"""This is a library of functions for performing color-based image segmentation of an image."""

import numpy as np
import cv2


def image_patch(img, x, y, w, h):
    """ Returns a region of interest of img specified by box """
    #check box against the boundaries of the image
    box = [y, x, y + h, x + w]
    if box[0] < 0:
        box[0] = 0
    if box[1] < 0:
        box[1] = 0
    if box[2] > img.shape[0]:
        box[2] = img.shape[0]
    if box[3] > img.shape[1]:
        box[3] = img.shape[1]

    return img[box[0]:box[2], box[1]:box[3], :]


def pixel_classify(p):
    """ Classify a pixel as background or foreground accoriding to a set of predefined rules """
    #This implementation is a stub. You should implement your own rules here.
    return 1.0


def image_classify(img):
    """ Classify each pixel in an image, and create a black-and-white mask """
    img_segmented = img.copy()
    for r in xrange(0, img.shape[0]):
        for c in xrange(0, img.shape[1]):
            p = img[r, c, :]
            if pixel_classify(p) < 0:
                img_segmented[r, c, :] = 0
            else:
                img_segmented[r, c, :] = 255
    return img_segmented


def image_line_vertical(img, x):
    """ Adds a green 3px vertical line to the image """
    cv2.line(img, (x, 0), (x, img.shape[1]), (0, 255, 0), 3)
    return img


def image_rectangle(img, x, y, w, h):
    """ Adds a green rectangle to the image """
    #This implementation is a stub. You should implement your own code here.

    return img


def image_one_to_three_channels(img):
    """ Transforms an image from two channels to three channels """
    #First promote array to three dimensions, then repeat along third dimension
    img_three = np.tile(img.reshape(img.shape[0], img.shape[1], 1), (1, 1, 3))
    return img_three


def test():
    #load sample image
    img = cv2.imread('../data/BU_logo.png', cv2.IMREAD_COLOR)
    #show sample region
    img_patch = image_patch(img, 50, 20, 20, 20)
    #run classifier to segment image
    img_segmented = image_classify(img)
    #add a line at 10px from the left edge
    img_segmented_line = image_line_vertical(img, 10)
    #show results
    cv2.imshow('patch', img_patch)
    cv2.imshow('segmented', img_segmented_line)
    cv2.waitKey(5000)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    test()
