#!/usr/bin/env python

import sys
import time
import rospy
from geometry_msgs.msg import Twist
from me416_lab.msg import MotorSpeedsStamped
from me416_lab import me416_utilities as mu
from me416_environment import motor_command_model as mcm
from std_msgs.msg import Float64

# The motors will most likely not spin exactly at the same speed, this is a simple factor to attempt to account for this.
# multiply the faster motor by this offset.
speed_offset = 0.95


class MotorManager():
    def __init__(self):
        self.pubSpeeds = rospy.Publisher('motor_speeds',
                                         MotorSpeedsStamped,
                                         queue_size=10)
        self.pubLeftSpeed = rospy.Publisher('/left_wheel_cmd',
                                         Float64,
                                         queue_size=10)
        self.pubRightSpeed = rospy.Publisher('/right_wheel_cmd',
                                         Float64,
                                         queue_size=10)
        rospy.Subscriber('robot_twist', Twist, self.translate_twist)
        # self.L_motor = mu.MotorSpeedLeft(speed_offset)
        self.L_motor = mu.MotorSpeedLeft()
        self.R_motor = mu.MotorSpeedRight()

    def translate_twist(self, twist_msg):
        if not rospy.is_shutdown():
            #extract rigid body velocities from message
            linear = twist_msg.linear.x
            angular = twist_msg.angular.z
            # rospy.loginfo("linear: {:f}, angular: {:f}".format(linear, angular))
            #mix velocities to compute motor speeds
            left, right = mcm.twist_to_speeds(linear, angular)
            #send motor speeds to the motors
            self.L_motor.set_speed(left)
            self.R_motor.set_speed(right)
            #prepare and publish motor speeds
            ms = MotorSpeedsStamped()
            ms.left = left
            ms.right = right
            ms.header.stamp = rospy.Time.now()
            self.pubSpeeds.publish(ms)
            self.pubLeftSpeed.publish(left)
            self.pubRightSpeed.publish(right)


def main():
    rospy.init_node('motors_node')
    mm = MotorManager()
    rospy.spin()


if __name__ == '__main__':
    main()
