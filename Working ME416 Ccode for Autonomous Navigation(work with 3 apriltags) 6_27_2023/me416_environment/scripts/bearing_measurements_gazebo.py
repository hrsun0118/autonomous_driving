#!/usr/bin/env python
""" turtlesim node: subscribe to receive turtle pose & publish bearings beta1 & beta2."""

import rospy
from me416_environment.msg import Bearings
from geometry_msgs.msg import PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
from math import pow, sqrt

def callback_tag_detections(msg):
    """Callback to receive a message from tag_detections
       Calculate beta0 & beta1"""
    global pub_bearings
    cam_to_apriltags_bearings = Bearings()

    # # print # of tags detected
    # if len(msg.detections) == 0:
    #     rospy.loginfo("No april tag is detected")
    # elif len(msg.detections) == 1:
    #     rospy.loginfo("Only 1 april tag is detected")
    # else:
    #     rospy.loginfo("2 april tags are detected")

    # Get bearings: 1) declare 3 arrays: numerators, denominators & bearings arrays
    # Note: for this apriltag setup in Gazebo world, index "element.id[0]" = 0 is left tag, "element.id[0]" = 1 is right tag
    n_xz_arr = np.zeros((len(msg.detections), 2))
    d_xz_arr = np.zeros(len(msg.detections))
    bearings_xz_arr = np.zeros((len(msg.detections), 2))
    # 2) For loop populate arrays (n_xz_arr, d_xz_arr, bearings_xz_arr)
    for i in range(len(msg.detections)):
        # normalize the vector into UNIT VECTOR
        # 2.1) get numerators' values from msg
        n_xz_arr[i][0] = msg.detections[i].pose.pose.pose.position.x
        n_xz_arr[i][1] = msg.detections[i].pose.pose.pose.position.z

        # 2.2) calculate the vector length
        d_xz_arr[i] = sqrt(pow(n_xz_arr[i][0], 2) + pow(n_xz_arr[i][1], 2))
        # 2.3) unit vector = vector / vector length
        bearings_xz_arr[i] = n_xz_arr[i] / d_xz_arr[i]
    # Print calculated bearings info on the terminal console - can be uncommented/commented during execution
    # for element in msg.detections:
    #     rospy.loginfo("beta{}_xz = [{:.2f},{:.2f}]".format(element.id[0][0], bearings_xz_arr[element.id[0]][0], bearings_xz_arr[element.id[0]][1]))

    # 3) put data into turtle_bearings
    # if len(msg.detections) == 1:     # publish zeros to bearings
    #     # turn left/right based on the ID
    cam_to_apriltags_bearings.len = len(msg.detections)
    if len(msg.detections) == 3 or len(msg.detections) == 2 or len(msg.detections) == 1:    # publish bearings for movements if 2 tags are detected
        cam_to_apriltags_bearings.beta0.x = bearings_xz_arr[0][0]
        cam_to_apriltags_bearings.beta0.y = bearings_xz_arr[0][1]
        if len(msg.detections) == 3 or len(msg.detections) == 2:
            cam_to_apriltags_bearings.beta1.x = bearings_xz_arr[1][0]
            cam_to_apriltags_bearings.beta1.y = bearings_xz_arr[1][1]
            if len(msg.detections) == 3:
                cam_to_apriltags_bearings.beta2.x = bearings_xz_arr[2][0]
                cam_to_apriltags_bearings.beta2.y = bearings_xz_arr[2][1]

        # rospy.loginfo("bearings.beta0: [%.3f][%.3f], bearings.beta1: [%.3f][%.3f], bearings.beta2: [%.3f][%.3f], ", cam_to_apriltags_bearings.beta0.x, cam_to_apriltags_bearings.beta0.y, cam_to_apriltags_bearings.beta1.x, cam_to_apriltags_bearings.beta1.y, cam_to_apriltags_bearings.beta2.x, cam_to_apriltags_bearings.beta2.y)
        pub_bearings.publish(cam_to_apriltags_bearings)



def main():
    global pub_bearings
    rospy.init_node('bearing_measurements_gazebo', anonymous=False)
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, callback_tag_detections)
    pub_bearings = rospy.Publisher("bearings", Bearings, queue_size = 10)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    finally:
        # "clean up" code: executed on shutdown in case of errors. E.g. closing files/windows
        pass
