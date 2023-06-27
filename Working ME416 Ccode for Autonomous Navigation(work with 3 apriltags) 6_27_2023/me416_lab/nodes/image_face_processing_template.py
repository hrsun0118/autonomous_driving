#!/usr/bin/env python
"""
Utilities for handling face detection
"""
import cv2
import image_processing as ip

#Path for xml file with cascade data
PATH_CASCADE_XML = '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml'

#Load cascade into global object
FACE_CASCADE = cv2.CascadeClassifier(PATH_CASCADE_XML)


def image_faces(img):
    """ Runs face detection on the image, and returns list of bounding boxes """
    faces = FACE_CASCADE.detectMultiScale(img, 1.3, 5)
    return faces


def image_faces_rectangle(img, faces):
    """ Adds a green rectangle around each detected face """
    print 'This implementation is a stub. Please insert here your code.'
    return img


def image_faces_patch_first(img, faces):
    """ Returns an image patch with the detected face """
    if len(faces) > 0:
        print 'This implementation is a stub. Please insert here your code.'
    else:
        return None


def test():
    img = cv2.imread('../data/president-george-washington.jpg')
    faces = image_faces(img)
    print 'faces: ', faces
    image_faces_rectangle(img, faces)
    img_patch = image_faces_patch_first(img, faces)
    cv2.imshow('patch', img_patch)
    cv2.imshow('detections', img)
    cv2.waitKey(5000)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    test()
