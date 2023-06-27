"""Functions for modeling ROSBot"""

import numpy as np
from math import cos, sin


def model_parameters():
    """Returns two constant model parameters"""
    k = 1.0
    d = 0.5
    return k, d


def system_matrix(theta):
    """Returns a numpy array with the A(theta) matrix for a differential drive robot"""
    k, d = model_parameters()
    A = (k / 2.0) * np.array([[m.cos(theta), m.cos(theta)],
                              [m.sin(theta), m.sin(theta)], [-1 / d, 1 / d]])
    return A


def euler_step(z, u, stepSize):
    """Integrates the dynamical model for one time step using Euler's method"""
    theta = z[2][0]
    A = system_matrix(theta)
    dot_z = A.dot(u)
    zp = z + stepSize * dot_z
    return zp


def clamp(v):
    """Clamp a value between -1 and 1"""
    return max(-1.0, min(1.0, v))


def twist_to_speeds(speed_linear, speed_angular):
    """Given the desired linear and angular speeds for the robot, returns the left and right motor speeds"""
    k, d = model_parameters()
    left = (speed_linear - (speed_angular * d)) / k
    right = (speed_linear + (speed_angular * d)) / k
    return clamp(left), clamp(right)


class KeysToVelocities(object):
    def __init__(self):
        self.speed_linear = 0.0
        self.speed_angular = 0.0
        self.SPEED_DELTA = 0.2
        self.last_action = ''

    def update_speeds(self, key):
        if key == 'w' or key == 'W':
            self.speed_linear += self.SPEED_DELTA
            self.last_action = 'Increased Linear Speed by %f' % (
                self.SPEED_DELTA)
        elif key == 's' or key == 'S':
            self.speed_linear -= self.SPEED_DELTA
            self.last_action = 'Decreased Linear Speed by %f' % (
                self.SPEED_DELTA)
        elif key == 'a' or key == 'A':
            self.speed_angular += self.SPEED_DELTA
            self.last_action = 'Increased Angular Speed by %f' % (
                self.SPEED_DELTA)
        elif key == 'd' or key == 'D':
            self.speed_angular -= self.SPEED_DELTA
            self.last_action = 'Decreased Angular Speed by %f' % (
                self.SPEED_DELTA)
        elif key == 'z' or key == 'Z':
            self.speed_linear = 0.0
            self.last_action = 'Zeroed Linear Speed'
        elif key == 'c' or key == 'C':
            self.speed_angular = 0.0
            self.last_action = 'Zeroed Angular Speed'
        elif key == 'x' or key == 'X':
            self.speed_angular = 0.0
            self.speed_linear = 0.0
            self.last_action = 'Zeroed All Speeds'
        else:
            self.last_action = 'Invalid Key Press'

        # Ensure published value is in range [-1,1]
        if self.speed_angular > 1.0:
            self.speed_angular = 1.0
        elif self.speed_angular < -1.0:
            self.speed_angular = -1.0

        if self.speed_linear > 1.0:
            self.speed_linear = 1.0
        elif self.speed_linear < -1.0:
            self.speed_linear = -1.0

        return self.speed_linear, self.speed_angular


class StampedMsgRegister(object):
    def __init__(self):
        self.msg_previous = None

    def replace_and_compute_delay(self, msg):
        if self.msg_previous == None:
            msg_previous = None
            self.msg_previous = msg
            time_delay = None
        else:
            msg_previous = self.msg_previous
            self.msg_previous = msg
            # time_delay = rospy.Time.to_sec(msg.header.stamp) - rospy.Time.to_sec(msg_previous.header)
            time_delay = msg.header.stamp.to_sec(
            ) - msg_previous.header.stamp.to_sec()

        return time_delay, msg_previous
