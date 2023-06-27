#!/usr/bin/env python

from __future__ import print_function

import math

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class GazeboRobot(object):
    def __init__(self,
                 robot_name="gazebo robot",
                 wheel_radius=1.0,
                 axis_distance=1.0):
        self.name = robot_name
        self.wheel_radius = wheel_radius
        self.axis_distance = axis_distance

        # Define how long before the velocities are set to 0 if there are no new messages
        self.MAX_UPDATE_DURATION_S = 20.0

        self.MAX_SPEED = 150.0 * math.pi * 2 / 60

        # Subscribers for the wheels
        self.right_wheel_subscriber = rospy.Subscriber(
            "/right_wheel_cmd", Float64, self.right_wheel_callback)
        self.left_wheel_subscriber = rospy.Subscriber("/left_wheel_cmd",
                                                      Float64,
                                                      self.left_wheel_callback)
        self.last_updated_right_wheel = rospy.get_time()
        self.last_updated_left_wheel = rospy.get_time()

        # Velocity publisher for Gazebo through /cmd_vel topic
        self.velocity_publisher = rospy.Publisher("/cmd_vel",
                                                  Twist,
                                                  queue_size=10)

        self.right_wheel_speed = 0.0
        self.left_wheel_speed = 0.0
        self.flag_new_data = False

    def right_wheel_callback(self, msg):
        self.right_wheel_speed = msg.data * self.MAX_SPEED
        self.last_updated_right_wheel = rospy.get_time()
        self.flag_new_data = True

    def left_wheel_callback(self, msg):
        self.left_wheel_speed = msg.data * self.MAX_SPEED
        self.last_updated_left_wheel = rospy.get_time()
        self.flag_new_data = True

    def check_last_update(self):
        # If we had new data, check to see if its expired
        if self.flag_new_data == True:
            currentTime = rospy.get_time()
            if (currentTime-self.last_updated_left_wheel > self.MAX_UPDATE_DURATION_S and \
                currentTime-self.last_updated_right_wheel > self.MAX_UPDATE_DURATION_S):
                self.flag_new_data = False
                self.right_wheel_speed = 0.0
                self.left_wheel_speed = 0.0
                rospy.loginfo("Setting speeds to 0")

    def update_robot(self):
        velocity_msg = Twist()

        # Linear robot velocity:
        velocity_msg.linear.x = -self.wheel_radius * (
            self.right_wheel_speed + self.left_wheel_speed) / 2
        # rospy.loginfo("Linear velocity: %f",velocity_msg.linear.x)

        # Angular robot velocity:
        velocity_msg.angular.z = -self.wheel_radius * (
            self.right_wheel_speed -
            self.left_wheel_speed) / (2 * self.axis_distance)
        # rospy.loginfo("Angular velocity: %f",velocity_msg.angular.z)

        self.velocity_publisher.publish(velocity_msg)

    def run(self):

        rate = rospy.Rate(60)

        rospy.loginfo("Gazebo robot running...")

        while not rospy.is_shutdown():
            self.check_last_update()
            self.update_robot()
            rate.sleep()


if __name__ == "__main__":

    rospy.init_node("gazebo_robot_node")

    wheel_radius = rospy.get_param("/wheel_radius", 1.0)
    axis_distance = rospy.get_param("/axis_distance", 1.0)

    gazeboRobot = GazeboRobot("Gazebo robot", wheel_radius, axis_distance)

    gazeboRobot.run()
