#!/usr/bin/env python
"""Utility to test ROSBot motors."""

import argparse
from me416_lab import me416_utilities as mu
import time

# The motors will most likely not spin exactly at the same speed, this is a simple factor to attempt to account for this.
# multiply the faster motor by this offset.
SPEED_FACTOR = 0.95


def apply_action(L_motor,
                 R_motor,
                 direction,
                 side,
                 duration=2,
                 speed=1,
                 is_recursive=True):
    """Apply the specified action to the specified side"""
    if direction == 'forward':
        pass
    elif direction == 'backward':
        speed = -speed
    elif direction == 'stop':
        speed = 0

    if side == 'left' or side == 'both':
        L_motor.set_speed(speed)

    if side == 'right' or side == 'both':
        R_motor.set_speed(speed)

    if is_recursive:
        time.sleep(duration)
        apply_action(L_motor, R_motor, 'stop', 'both', is_recursive=False)


def main(args):
    """Create motor objects, then apply specified commands."""

    L_motor = mu.MotorSpeedLeft(SPEED_FACTOR)
    R_motor = mu.MotorSpeedRight()

    apply_action(L_motor, R_motor, args.direction, args.side, args.duration,
                 args.speed)


if __name__ == "__main__":
    example_text = """example usage: ./motor_utility.py forward --side left --duration 5"""
    parser = argparse.ArgumentParser(
        description='Utility to test ROSBot motors.',
        epilog=example_text,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('direction')
    parser.add_argument('--side',
                        type=str,
                        default='both',
                        help='Side to run, can be "left", "right", or "both"')
    parser.add_argument('--duration',
                        type=int,
                        default=2,
                        help='Duration of the motor activation')
    parser.add_argument('--speed',
                        type=float,
                        default=2,
                        help='Normalized speed between 0 and 1')
    args = parser.parse_args()
    main(args)
