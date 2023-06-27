#!/usr/bin/env python
""" Generate a time-varying error signal on the topics:
 - /error_signal_stamped, which is of type PointStamped (error is in the point.x field)
 - /error_signal, which is of type Float64
"""

import rospy

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64
import math

SIGNAL_FREQUENCY = 1


def main():
    """Node setup and main ROS loop"""
    rospy.init_node('signal_generator')

    #Set rate to use (in Hz)
    rate = rospy.Rate(50)

    #Publisher
    pub_stamped = rospy.Publisher(
        'error_signal_stamped', PointStamped, queue_size=2)
    pub = rospy.Publisher('error_signal', Float64, queue_size=2)
    msg = PointStamped()
    while not rospy.is_shutdown():
        #Get current time and the value of a cosine wave
        current_time = rospy.Time.now()
        error_signal = math.cos(
            2 * math.pi * SIGNAL_FREQUENCY * current_time.to_sec())

        #Publish on the stamped topic
        msg.header.stamp = current_time
        msg.point.x = error_signal
        pub_stamped.publish(msg)

        #Publish on the non-stamped topic
        pub.publish(Float64(error_signal))

        #Wait to achieve target rate
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        pass
