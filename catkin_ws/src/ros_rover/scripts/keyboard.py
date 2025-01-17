#!/usr/bin/env python

"""
Keyboard node.
"""

import select
import sys
import termios
import tty

import rospy

from ros_rover.msg import Teleoperation

# Speed values
SPEED_STEP_SIZE = 5
SPEED_MIN = -100
SPEED_ZERO = 0
SPEED_MAX = 100
# Angle values
ANGLE_STEP_SIZE = 5
ANGLE_MIN = -100
ANGLE_ZERO = 0
ANGLE_MAX = 100

msg = """
Rover Control Panel!
------------------------------------------------------
Moving around:
    q (max front speed) w (forward) e (align)
    a (left)            s (stop)    d (right)
    z (max back speed)  x (back)    c (max left)   v (max right)

w/x : increase/decrease speed ( ~ 0.05 )
a/d : increase/decrease angle (~ 5 grades )

space key, s : force stop

CTRL-C to quit
"""


def get_key():
    """
    Return key pressed
    """

    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    return key


def print_movement(key, t_speed, t_angle):
    """
    Print a string with the current speed and angle (degrees and value)
    """

    print "%s pressed\ncurrently:\tspeed %s\t angle %s" %(key, t_speed, t_angle)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    # Register client node with the master under keyboard_node name
    rospy.init_node("keyboard_node")
    # Register keyboard_node node as a publisher of rover_motion ROS topic
    pub = rospy.Publisher("teleop_topic", Teleoperation, queue_size=1)

    status = 0
    target_speed = SPEED_ZERO
    target_angle = ANGLE_ZERO

    try:
        print msg

        while 1:
            key_pressed = get_key().upper()

            if key_pressed == 'W':
                # Go Forward
                # If target_speed did not have reached the +SPEED_MAX
                # or if it has reached the -SPEED_MAX
                if abs(target_speed) < SPEED_MAX or target_speed == -SPEED_MAX:
                    target_speed = target_speed + SPEED_STEP_SIZE

                print_movement(key_pressed, target_speed, target_angle)
                status = status + 1
            elif key_pressed == 'S':
                # Stop
                target_speed = SPEED_ZERO
                print_movement(key_pressed, target_speed, target_angle)
                status = status + 1
            elif key_pressed == 'X':
                # Go Back
                # If target_speed did not have reached the -SPEED_MAX
                # or if it has reached the +SPEED_MAX
                if abs(target_speed) < SPEED_MAX or target_speed == SPEED_MAX:
                    target_speed = target_speed - SPEED_STEP_SIZE

                print_movement(key_pressed, target_speed, target_angle)
                status = status + 1
            elif key_pressed == 'A':
                # Turn Left
                if target_angle > ANGLE_MIN:
                    target_angle = target_angle - ANGLE_STEP_SIZE

                print_movement(key_pressed, target_speed, target_angle)
                status = status + 1
            elif key_pressed == 'D':
                # Turn Right
                if target_angle < ANGLE_MAX:
                    target_angle = target_angle + ANGLE_STEP_SIZE

                print_movement(key_pressed, target_speed, target_angle)
                status = status + 1
            elif key_pressed == 'E':
                # Align Wheels
                target_angle = ANGLE_ZERO
                print_movement(key_pressed, target_speed, target_angle)
                status = status + 1
            elif key_pressed == 'Q':
                # Go Forward (+SPEED_MAX)
                target_speed = SPEED_MAX
                print_movement(key_pressed, target_speed, target_angle)
                status = status + 1
            elif key_pressed == 'Z':
                # Go Back (-SPEED_MAX)
                target_speed = -SPEED_MAX
                print_movement(key_pressed, target_speed, target_angle)
                status = status + 1
            elif key_pressed == 'C':
                # Turn Left (ANGLE_MIN)
                target_angle = ANGLE_MIN
                print_movement(key_pressed, target_speed, target_angle)
                status = status + 1
            elif key_pressed == 'V':
                # Turn Right (ANGLE_MAX)
                target_angle = ANGLE_MAX
                print_movement(key_pressed, target_speed, target_angle)
                status = status + 1
            else:
                if key_pressed == '\x03':
                    # Break infinite while loop
                    print "Control + C pressed"
                    break

            # After 20 keystrokes show the Rover Control Panel
            if status == 20:
                print msg
                status = 0

            # Instantiate and publish a ROS Message (Teleoperation)
            teleoperation = Teleoperation()
            teleoperation.speed = target_speed
            teleoperation.steering = target_angle
            pub.publish(teleoperation)

    except:
        # Exceptions
        print "Communications Failed"

    finally:
        # Instantiate and publish a ROS Message (Teleoperation)
        # Stop (SPEED_ZERO) and align the wheels (ANGLE_ZERO)
        print "Finally"
        teleoperation = Teleoperation()
        teleoperation.speed = SPEED_ZERO
        teleoperation.steering = ANGLE_ZERO
        pub.publish(teleoperation)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
