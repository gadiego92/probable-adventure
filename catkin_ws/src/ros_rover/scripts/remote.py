#!/usr/bin/env python

"""
Remote node.
"""

import math

import rospy
from sensor_msgs.msg import Joy

from ros_rover.msg import Teleoperation


def callback(joy_message):
    """
    Callback function called when a Joy message is received from /joy topic

    :param list joy_message: List with Joy parameters
    """

    # LEFT
    # Left 	[0] +
    # Right [0] -
    # Front	[1] +
    # Back	[1] -

    left_x = -joy_message.axes[0]
    left_y = joy_message.axes[1]
    left_trigger = joy_message.axes[2]
    square_button = joy_message.buttons[3]

    teleoperation = Teleoperation()
    teleoperation.speed = set_speed(left_trigger, square_button)
    teleoperation.steering = set_steering(left_x, left_y)

    pub.publish(teleoperation)


def set_speed(left_trigger, square_button):
    """
    Set speed [-100, +100] from right joystick values [x, y]

    :param float left_trigger: Y axe joystick value [-1, 1]
    :param int square_button: Square button 0 or 1
    """

    old_min = -1
    old_max = 1

    # [-1, 1] to [0, 100]
    tmp_y = (1 - left_trigger) / 2

    # Normalize speed value
    speed = normalize_values_100(tmp_y, old_min, old_max)

    # if square button is pressed, change sign to go back
    if square_button:
        speed *= -1

    return speed


def set_steering(x, y):
    """
    Set radius [-100, +100] from left joystick values [x, y]

    :param float x: X axe joystick value [-1, 1]
    :param float y: Y axe joystick value [-1, 1]
    """

    old_min = 0
    old_max = 180

    # If there is not steering don't calculate anything
    if x == 0 and y == 0:
        return 0

    # Convert cartesian coordinates to polar coordinates
    # Ignore polar return value
    _, theta = cartesian2polar(x, y)

    # Convert negative angle to corresponding positive
    if theta < 0:
        theta = -theta

    # Normalize radius angle value
    radius = normalize_values_100(theta, old_min, old_max)

    # Change value sign [left(-), right(+)]
    return -radius


def cartesian2polar(x, y):
    """
    Convert cartesian coordinates to polar coordinates

    :param float x: X axe joystick value
    :param float y: Y axe joystick value
    """

    polar = math.sqrt(math.pow(x, 2) + math.pow(y, 2))

    try:
        # Get angle from 2-argument arctangent
        theta = math.degrees(math.atan2(y, x))
    except:
        # Go front [0 - 180]
        theta = 90

    return polar, theta


def normalize_values_100(old_value, old_min, old_max):
    """
    Normalize an old_value [old_min, old_max] to a new value [-100, 100]

    :param float old_value: value to be normalize
    :param int old_min: old minimum interval value
    :param int old_max: old maximum interval value
    """

    new_min = -100
    new_max = 100

    return (new_max - new_min) * ((old_value - old_min) / (old_max - old_min)) + new_min


if __name__ == "__main__":
    global pub

    rospy.init_node("remote_node")
    rospy.loginfo("Starting the remote node")

    sub = rospy.Subscriber("joy", Joy, callback)
    pub = rospy.Publisher("teleop_topic", Teleoperation, queue_size=1)

    rospy.spin()
