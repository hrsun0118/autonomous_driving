#!/usr/bin/env python
"""
Publishes the values of the encoders
"""

import rospy
import me416_utilities as mu
from me416_lab.msg import MotorSpeedsStamped


def main():
    """Node setup and main ROS loop"""
    #Init node. anonymous=True allows multiple launch with automatically assigned names
    rospy.init_node('encoders_publisher', anonymous='True')

    #Prepare publisher on the 'chatter' topic
    pub = rospy.Publisher('motor_speeds_encoders',
                          MotorSpeedsStamped,
                          queue_size=10)

    #Prepare objects for message and encoders
    msg = MotorSpeedsStamped()
    encoder_right = mu.QuadEncoderRight(updateInterval=None)
    encoder_left = mu.QuadEncoderLeft(updateInterval=None)

    #Set rate to use (in Hz)
    rate = rospy.Rate(3)

    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        msg.left = encoder_left.get_velocity() / 15000
        msg.right = encoder_right.get_velocity() / 15000
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        pass
