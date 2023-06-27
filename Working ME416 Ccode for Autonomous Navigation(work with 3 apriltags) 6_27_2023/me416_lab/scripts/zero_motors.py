#!/usr/bin/env python

from me416_lab import me416_utilities as mu

# The motors will most likely not spin exactly at the same speed, this is a simple factor to attempt to account for this.
# multiply the faster motor by this offset.
speed_offset = 0.95

L_motor=mu.MotorSpeedLeft(speed_offset)
R_motor=mu.MotorSpeedRight()

def main():
    L_motor.set_speed(0.0)
    R_motor.set_speed(0.0)

if __name__ == '__main__':
    main()
