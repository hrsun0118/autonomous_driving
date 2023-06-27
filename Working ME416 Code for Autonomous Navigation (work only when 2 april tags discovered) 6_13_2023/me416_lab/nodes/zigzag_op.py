#!/usr/bin/env python
""" Node that outputs a sequence of forward-moving arches """

import rospy
from geometry_msgs.msg import Twist


class ZigZagCommander(object):
    """
    Publishes Twist commands on robot_twist topic corresponding to a
    sequence of forward-moving arches
    """

    def __init__(self):
        self.pub = rospy.Publisher('robot_twist', Twist, queue_size=10)
        self.count = 0
        self.nb_actions = 2

    def send(self):
        """ Publishes a new Twist command """
        #Init message. It is a 3-D Twist, but we will use only two fields
        msg = Twist()

        #Fill in linear (x-axis) and angular (z-axis) velocities
        msg.linear.x = 0.5
        if self.count == 0:
            msg.angular.z = -0.5
        else:
            msg.angular.z = 0.5

        #Update count
        self.count = (self.count + 1) % self.nb_actions

        #Publish
        self.pub.publish(msg)


def main():
    """ Main ROS loop """
    rospy.init_node('zigzag_op')
    #Set rate to use (in Hz)
    rate = rospy.Rate(1)
    zzc = ZigZagCommander()
    while not rospy.is_shutdown():
        #Talk
        zzc.send()
        #Wait until it is done
        rate.sleep()


if __name__ == '__main__':
    main()
