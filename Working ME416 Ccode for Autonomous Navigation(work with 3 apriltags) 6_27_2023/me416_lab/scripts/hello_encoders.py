#!/usr/bin/env python
"""
Test script to read encoders
"""

from me416_lab import me416_utilities as mu
import time
import signal


class ServiceExit(Exception):
    """
    Custom exception to handle sigint
    """
    pass


def exit_gracefully(signum, frame):
    raise ServiceExit


def main():
    signal.signal(signal.SIGINT, exit_gracefully)
    signal.signal(signal.SIGTERM, exit_gracefully)
    encoderRight = mu.QuadEncoderRight(updateInterval=None)
    encoderLeft = mu.QuadEncoderLeft(updateInterval=None)
    try:
        for i in range(1000):
            print('Right Velocity: ' + str(encoderRight.get_velocity()) +
                  ' Left Velocity: ' + str(encoderLeft.get_velocity()))
            time.sleep(0.5)
    except ServiceExit:
        # Shut down signal received
        print('Stopping')


if __name__ == "__main__":
    main()
