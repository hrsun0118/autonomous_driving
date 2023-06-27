"""
This is a library of helpful classes and functions for the ME416 Lab. If the module is on non-RasberryPi systems (more
exactly, where the RPi module is not available), the motor commands are logged to the console
"""

from __future__ import print_function
from threading import Thread, Event
import atexit
import time

# This module might be used not on a RPi (i.e., without GPIO)
# The global constant will allow to know if we can use the GPIO or not
try:
    import RPi.GPIO as GPIO
except ImportError:
    IS_RPI = False
else:
    IS_RPI = True

# Set the pin numbering scheme to board numbers (as opposed to Broadcom number, see the pinout)
if IS_RPI:
    GPIO.setmode(GPIO.BOARD)

# Wrappers for setting pins and cleanup. These are used so that we can cleanup only when we actually set some pins (avoid warning messages)
IS_SETUP = False


def setup(pin, mode):
    global IS_SETUP
    if IS_RPI:
        GPIO.setup(pin, mode)
        IS_SETUP = True


def cleanup():
    if IS_SETUP and IS_RPI:
        GPIO.cleanup()


# Register the cleanup function to run at exit
if IS_RPI:
    atexit.register(cleanup)

# Assign variable names to the pins so you don't have to control them by number
R_forward_pin = 31
R_backward_pin = 29
L_forward_pin = 16
L_backward_pin = 18
# Define the GPIO pins for reading quadrature encoder
R_encoder_A = 19
R_encoder_B = 21
L_encoder_A = 3
L_encoder_B = 5


#Class to read quadrature encoders
class QuadEncoder(object):
    """
    A class to read the two output of a quadrature encoder and estimate its speed through GPIO.
    If the updateInterval is set to None, the velocity is computed by averaging between calls to get_velocity; otherwise, it is updates every updateInterval seconds, using a daemonic thread to update it.
    ENCODER LOGIC: We assume that positive direction is when A is the leading edge. Thus, whenever we see
    a edge transition we check if the triggering pin has the same state as the non-triggering pin (trigger pin is following)
    or opposite (trigger pin is leading). For our 150RPM motor @4.5V with 120:1 gear ratio and encoder with 12CPR, we expect a max CPR of 150/60*120*12=3600.
    """
    def __init__(self,
                 A_pin,
                 B_pin,
                 updateInterval,
                 encoder_name="quadrature"):
        self.encoder_name = encoder_name
        self.updateInterval = updateInterval

        # Setup pins and routines for encoder counters
        self.count = 0
        self.A_pin = A_pin
        self.B_pin = B_pin
        if IS_RPI:
            setup(self.A_pin, GPIO.IN)
            setup(self.B_pin, GPIO.IN)
            # Get the initial pin states
            self.A_state = GPIO.input(self.A_pin)
            self.B_state = GPIO.input(self.B_pin)
            # Add interrupts for the encoder reading
            time.sleep(0.01)
            GPIO.add_event_detect(self.A_pin,
                                  GPIO.BOTH,
                                  callback=self.A_callback)
            time.sleep(0.01)
            GPIO.add_event_detect(self.B_pin,
                                  GPIO.BOTH,
                                  callback=self.B_callback)
        else:
            self.A_state = True
            self.B_state = True
            print('Encoder "%s" initialized' % encoder_name)

        if updateInterval is None:
            # Update velocity only when requested
            self.thread = None
            self.event = None
        else:
            # Init a Thread and an Event to stop it
            self.thread = Thread(target=self.run)
            self.event = Event()
            # Run the thread as daemonic, so that it will terminate when the main thread stops
            self.thread.daemon = True
            # Run thread
            self.thread.start()

        self.velocity = 0
        self.lastUpdateTime = time.clock()

    def A_callback(self, channel):
        self.A_state = GPIO.input(self.A_pin)
        if self.A_state == self.B_state:
            self.count -= 1  # A follows B
        else:
            self.count += 1  # A leads B

    def B_callback(self, channel):
        self.B_state = GPIO.input(self.B_pin)
        if self.A_state == self.B_state:
            self.count += 1  # B follows A
        else:
            self.count -= 1  # B leads A

    # Thread's run() function to compute the encoder speed as counts/second
    def run(self):
        while not self.event.is_set():
            self.update_velocity()
            time.sleep(self.updateInterval)

    def update_velocity(self):
        currentTime = time.clock()
        self.velocity = float(self.count) / (currentTime - self.lastUpdateTime)
        self.count = 0
        self.lastUpdateTime = currentTime

    # Return the velocity in counts/seconds
    def get_velocity(self):
        if self.updateInterval is None:
            self.update_velocity()
        return self.velocity

    # Function to update the interval
    def set_interval(self, newInterval):
        self.updateInterval = newInterval

    # Function to stop thread
    def stop(self):
        self.event.set()

    # Destructor
    def __del__(self):
        """ Destructor: ask thread to stop """
        self.event.set()


# Specialized class for left and right encoders
class QuadEncoderRight(QuadEncoder):
    """Specialized class to create a right encoder"""
    def __init__(self, updateInterval=0.1):
        QuadEncoder.__init__(self, R_encoder_A, R_encoder_B, updateInterval,
                             "Right Encoder")


class QuadEncoderLeft(QuadEncoder):
    """Specialized class to create a left encoder"""
    def __init__(self, updateInterval=0.1):
        QuadEncoder.__init__(self, L_encoder_A, L_encoder_B, updateInterval,
                             "Left Encoder")


# Motor control class
class MotorSpeed:
    """A class to control motors using PWM on all channels of an H-bridge thorugh GPIO"""
    def __init__(self,
                 fw_pin,
                 bw_pin,
                 speed_factor=1.0,
                 max_duty_cycle=90,
                 motor_name="Motor"):
        #Save parameter privately
        self.speed_factor = speed_factor
        self.max_duty_cycle = 90
        self.motor_name = motor_name

        #Init pins and PWM objects
        if IS_RPI:
            setup(fw_pin, GPIO.OUT)
            setup(bw_pin, GPIO.OUT)
            self.fw_pwm = GPIO.PWM(fw_pin, 100)
            self.bw_pwm = GPIO.PWM(bw_pin, 100)
            self.fw_pwm.start(0)
            self.bw_pwm.start(0)
        else:
            print('Motor "%s" initialized' % motor_name)

    def set_speed(self, speed):
        """ Set speed. speed=-1 is max_duty_cycle backward, speed=1 is max_duty_cycle foward, speed=0 is stop """
        duty_cycle = min(
            int(abs(speed) * self.speed_factor * self.max_duty_cycle),
            self.max_duty_cycle)
        if speed < 0:
            duty_cycle_bw = duty_cycle
            duty_cycle_fw = 0
        elif speed > 0:
            duty_cycle_bw = 0
            duty_cycle_fw = duty_cycle
        else:
            duty_cycle_bw = 0
            duty_cycle_fw = 0

        if IS_RPI:
            self.bw_pwm.ChangeDutyCycle(duty_cycle_bw)
            self.fw_pwm.ChangeDutyCycle(duty_cycle_fw)
        else:
            print('%s duty cycles: Forward = %d, Backward = %d.' %
                  (self.motor_name, duty_cycle_fw, duty_cycle_bw))


#Specialized motor classes
class MotorSpeedLeft(MotorSpeed):
    """Inherited class specialized to left motor"""
    def __init__(self, speed_factor=1.0, max_duty_cycle=90):
        MotorSpeed.__init__(self,
                            L_forward_pin,
                            L_backward_pin,
                            speed_factor,
                            max_duty_cycle,
                            motor_name="Left motor")


class MotorSpeedRight(MotorSpeed):
    """Inherited class specialized to left motor"""
    def __init__(self, speed_factor=1.0, max_duty_cycle=90):
        MotorSpeed.__init__(self,
                            R_forward_pin,
                            R_backward_pin,
                            speed_factor,
                            max_duty_cycle,
                            motor_name="Right motor")


# Keyboard functions
class _Getch(object):
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self):
        return self.impl()


class _GetchUnix(object):
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows(object):
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


# CSV reading
def read_two_columns_csv(filename):
    """
    Read a CSV (Comma Separated Values) file with numerical values,
    and return a list of lists with the contents of the first two columns of the file.
    If there is an error in opening the file, the returned list is empty.
    If a row in the file contains less than two, it is skipped.
    """
    import numpy as np

    pair_list = []
    with open(filename, 'r') as file_id:
        #use one of NumPy functions to load the data into an array
        data = np.genfromtxt(file_id, delimiter=',')
        #iterate over the rows
        for row in data:
            if len(row) >= 2:
                #append the content of the first two columns to the list
                pair_list.append([row[0], row[1]])
    return pair_list
