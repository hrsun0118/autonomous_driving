#!/usr/bin/env python
'''Node that outputs a sequence of zero-velocity commands
'''

import rospy
from geometry_msgs.msg import Twist


def main():
    rospy.init_node('null_op')
    rate = rospy.Rate(1)

    msg = Twist()

    while not rospy.is_shutdown():
        #Send command
        pub.publish(msg)
        #Wait to reach target rate
        rate.sleep()


if __name__ == '__main__':
    main()
