#!/usr/bin/env python
""" Node that outputs a sequence of forward-moving arches """

import rospy
from geometry_msgs.msg import Twist


class SquareCommander(object):
    """
    Publishes Twist commands on robot_twist topic corresponding to a
    sequence of forward-moving arches
    """

    def __init__(self):
        self.pub = rospy.Publisher('robot_twist', Twist, queue_size=10)
        self.count = 0
        self.nb_actions = 4

    def send(self):
        """ Publishes a new Twist command """
        #Init message. It is a 3-D Twist, but we will use only two fields
        msg = Twist()

        #Fill in linear (x-axis) and angular (z-axis) velocities
        if self.count == 0:
            msg.linear.x = 1
            msg.angular.z = 0
        elif self.count == 2:
            msg.linear.x = 0
            msg.angular.z = 1
        else:
            msg.linear.x = 0
            msg.angular.z = 0

        #Update count
        self.count = (self.count + 1) % self.nb_actions

        #Publish
        self.pub.publish(msg)


def main():
    """ Main ROS loop """
    rospy.init_node('square_op')
    #Set rate to use (in Hz)
    delay = 2  #seconds
    rate = rospy.Rate(1.0 / delay)
    zzc = SquareCommander()
    while not rospy.is_shutdown():
        #Talk
        zzc.send()
        #Wait until it is done
        rate.sleep()


if __name__ == '__main__':
    main()
