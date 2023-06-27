#!/usr/bin/env python
"""Simple talker demo that published std_msgs/Strings messages to the 'chatter' topic
Wraps the "talker" into an object, and show how to use an "internal state"
"""

import rospy
from std_msgs.msg import String


class Talker():
    """Class that repeatedly counts from 0 to 9 and publishes to the topic chatter"""

    def __init__(self):
        #Init publisher on the 'chatter' topic
        self.pub = rospy.Publisher('chatter', String, queue_size=10)
        self.count = 0

    def updateCount(self):
        """Update internal counter modulo 10"""
        #Count from 0 to 9 and then repeat
        self.count += 1
        if self.count > 9:
            self.count = 0
        #The above could be simplified as
        #self.count=(self.count+1)%10

    def talk(self):
        """Update the count, write the message on console and on the topic"""
        #Prepare message object
        msg = String()

        #Prepare string
        msg.data = 'I am %s. I am counting %d.' % [
            rospy.get_name(), self.count()
        ]

        #Write to console
        rospy.loginfo(msg.data)

        #Publish
        self.pub.publish(msg.data)


def main():
    """Setup node and talker object. Main ROS loop."""
    #Init node. anonymous=True allows multiple launch with automatically assigned names
    rospy.init_node('talker', anonymous='True')
    #Set rate to use (in Hz)
    rate = rospy.Rate(1)
    #to stands for tALKER oBJECT
    to = Talker()
    while not rospy.is_shutdown():
        #Talk
        to.talk()
        #Update count
        to.updateCount()
        #Wait until it is done
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        pass
