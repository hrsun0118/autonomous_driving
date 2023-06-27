#!/usr/bin/env python
""" Node that outputs velocities from the keyboard inputs"""
from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist
from me416_environment import motor_command_model as mcm
from me416_lab import me416_utilities as mu

def main():
    """The node permits to change velocities of the robot through the press of certain keys on your keyboard:
    press:
    w - Increases linear velocity
    a - Decrease angular velocity (turn left)
    s - Decrease linear velocity
    d - Increase angular velocity (turn right)
    q - Quit
    """

    rospy.init_node('key_op')
    pub = rospy.Publisher('robot_twist', Twist, queue_size=10)
    msg = Twist()

    k2v = mcm.KeysToVelocities()

    getch = mu._Getch()

    while not rospy.is_shutdown():
        key = getch()
        if key == 'q':
            rospy.loginfo('Shutdown Initiated')
            rospy.signal_shutdown('Shutting down initiated by %s' %
                                  rospy.get_name())
        else:
            linear, angular = k2v.update_speeds(key)
            print(k2v.last_action)

            msg.linear.x = linear
            msg.angular.z = angular
            pub.publish(msg)


if __name__ == '__main__':
    try:
        main()
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        pass
