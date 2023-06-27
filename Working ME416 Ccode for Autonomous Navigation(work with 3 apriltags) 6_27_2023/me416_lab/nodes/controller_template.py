#!/usr/bin/env python
""" A first template for a PID controller """

class PID(object):
    """ Computes proportional, derivative, and integral terms for a PID controller """

    def __init__(self, kp=1.0, kd=1.0, ki=1.0):
        """Initializes gains and internal state (previous error and integral error)"""
        self.kp = kp
        self.kd = kd
        self.ki = ki

        self.error_signal_previous = None
        self.error_signal_integral = 0

    def proportional(self, error_signal):
        """ Compute proportional term (with gain) """
        #TODO: This is a stub. Write your code here.
        return f_b

    def integral(self, error_signal, time_delay):
        """ Compute integral term (with gain) """
        #TODO: This is a stub. Write your code here.
        return f_i

    def derivative(self, error_signal, time_delay):
        """ Compute derivative term (with gain) """
        #TODO: This is a stub. Write your code here.
        return f_d

