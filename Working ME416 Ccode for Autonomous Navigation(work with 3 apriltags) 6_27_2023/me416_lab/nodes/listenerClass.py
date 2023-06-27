#!/usr/bin/env python
"""Simple listener demo that listens to std_msgs/Strings published to the 'chatter' topic.
Wraps the listener in a class
"""

import rospy
from std_msgs.msg import String


class Listener:
    """Class that subscribes on the topic chatter and echoes what it receives"""

    def __init__(self):
        #Use the 'chatter' topic
        rospy.Subscriber('chatter', String, self.callback)

    def callback(self, msg):
        """Callback for the subscriber"""
        rospy.loginfo(' I heard %s', msg.data)


def listener():
    """Setup node and listener object"""
    #Init node. anonymous=True allows multiple launch with automatically assigned names
    rospy.init_node('listener', anonymous=True)
    #Create the listener object (it will setup its own callback
    lo = Listener()
    #Run until stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    finally:
        #This is the place to put any "clean up" code that should be executed
        #on shutdown even in case of errors, e.g., closing files or windows
        pass
