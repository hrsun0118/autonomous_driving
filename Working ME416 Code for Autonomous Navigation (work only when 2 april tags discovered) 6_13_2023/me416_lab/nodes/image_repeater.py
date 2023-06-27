#!/usr/bin/env python
'''
Simple node that repeats images from a compressed topic to a non-compressed one.
'''

import rospy
import cv2
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import numpy as np


class ImageRepeater():
    def __init__(self):
        rospy.Subscriber(
            'raspicam_node/image/compressed',
            CompressedImage,
            callback=self.callback,
            queue_size=1,
            buff_size=2**18
        )  #the buff_size=2**18 avoids delays due to the queue buffer being too small for images
        self.pub = rospy.Publisher('image_repeated', Image, queue_size=1)
        self.bridge = CvBridge()

    def callback(self, msg):
        # Convert ros image into a cv2 image (we cannot use bridge because topic is compressed)
        np_arr = np.fromstring(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img = cv2.flip(img, -1)

        #The following line would be for non-compressed images
        #img=bridge.imgmsg_to_cv2(msg,"bgr8")
        img_processed = img.copy()
        self.pub.publish(self.bridge.cv2_to_imgmsg(img_processed, "bgr8"))


if __name__ == '__main__':
    rospy.init_node('image_repeater')
    ir = ImageRepeater()
    rospy.spin()
