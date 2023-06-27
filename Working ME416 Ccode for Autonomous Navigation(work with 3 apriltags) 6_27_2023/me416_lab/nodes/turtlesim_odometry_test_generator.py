#!/usr/bin/env python
"""
Node to publish time-varying poses to pose_euler and pose_arcs for testing
"""

import rospy
import math
from geometry_msgs.msg import Pose2D

class PoseMsgGenerator(object):
    def __init__(self,r,omega):
        self.msg=Pose2D()
        self.r=r
        self.omega=omega

    def pose_now(self):
        t=rospy.Time.now().to_sec()
        theta=math.fmod(self.omega*t,2*math.pi)
        self.msg.x=self.r*math.sin(theta)
        self.msg.y=-self.r*math.cos(theta)+self.r
        self.msg.theta=theta

        return self.msg

def main():
    """Node setup and use"""
    # Initialize node
    rospy.init_node('controller_test_node')

    pose_list=['euler','arcs']
    pose_pub=dict((s,rospy.Publisher('/pose_'+s,Pose2D,queue_size=10)) for s in pose_list)
    pose_gen=dict((s,PoseMsgGenerator(i,i)) for (i,s) in enumerate(pose_list,start=1))
    rate=rospy.Rate(10)

    while not rospy.is_shutdown():
        for s in pose_list:
            msg=pose_gen[s].pose_now()
            pose_pub[s].publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        pass
