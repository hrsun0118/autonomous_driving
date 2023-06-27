#!/usr/bin/env python
"""
Node to publish time-varying poses to pose_euler and pose_arcs for testing
"""
from __future__ import print_function

import rospy
import math
from geometry_msgs.msg import Pose2D
from turtlesim.srv import TeleportAbsolute

class TurtlePoseTeleporter(object):
    def __init__(self, name):
        self.name=name
        self.service_name='/turtle_'+self.name+'/teleport_absolute'
        rospy.Subscriber('/pose_'+name, Pose2D, self.callback)

    def service_wait(self):
        rospy.loginfo('Waiting for service '+self.service_name)
        rospy.wait_for_service(self.service_name)
        rospy.loginfo('Service '+self.service_name+' available')

    def callback(self,msg):
        service = rospy.ServiceProxy(self.service_name, TeleportAbsolute)
        try:
            service(msg.x+5, msg.y+5, msg.theta)
        except rospy.ServiceException as exc:
            rospy.error("Service did not process request: " + str(exc))

def main():
    """Node setup and use"""
    # Initialize node
    rospy.init_node('controller_test_node')

    turtle_list=['euler','arcs']
    turtle_teleporter=dict((s,TurtlePoseTeleporter(s)) for s in turtle_list)

    for name in turtle_list:
        turtle_teleporter[name].service_wait()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        pass
