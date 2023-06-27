#!/usr/bin/env python
""" Node that applies the PIDController from control.py to the topic /error_signal
"""

import rospy
import controller

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64


class ControllerTest(object):
    def __init__(self):
        # Use the 'chatter' topic
        rospy.Subscriber('/error_signal_stamped', PointStamped, self.callback)
        self.pub = {}
        for pub_name in ['proportional', 'derivative', 'integral']:
            self.pub[pub_name] = rospy.Publisher(
                '/control_' + pub_name, Float64, queue_size=2)
        self.stamp_previous = None
        self.pid = controller.PIDController()

    def callback(self, msg):
        error_signal = msg.point.x
        stamp = msg.header.stamp

        control_proportional = self.pid.proportional(error_signal)
        msg_proportional = Float64(control_proportional)
        self.pub['proportional'].publish(msg_proportional)
        if self.stamp_previous:
            time_delay = (stamp - self.stamp_previous).to_sec()

            control_derivative = self.pid.derivative(error_signal, time_delay)
            msg_derivative = Float64(control_derivative)
            self.pub['derivative'].publish(msg_derivative)

            control_integral = self.pid.integral(error_signal, time_delay)
            msg_integral = Float64(control_integral)
            self.pub['integral'].publish(msg_integral)

        self.stamp_previous = stamp


def main():
    """Node setup and use"""
    # Initialize node
    rospy.init_node('controller_test_node')
    ct = ControllerTest()

    # Run until stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        pass
