#!/usr/bin/env python
"""Simple script that waits for the key 'q' and then terminates"""

import rospy
import me416_utilities as mu


def main():
    """Function to setup node and loop"""
    #Setup node and object for rate throttling
    rospy.init_node('key_emergency_switch')
    rate = rospy.Rate(50)
    print '     Press "q" to quit.\n'

    #Boilerplate to get function to read keyboard
    getch = mu._Getch()

    #Main ROS loop
    while not rospy.is_shutdown():
        key = getch()
        if key == 'q':
            rospy.loginfo("Shutdown initiated")
            rospy.signal_shutdown(
                'Shutting down initiated by %s' % rospy.get_name())
        rate.sleep()


if __name__ == '__main__':
    main()
