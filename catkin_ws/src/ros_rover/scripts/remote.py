#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from ros_rover.msg import Teleoperation

import math


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

    # RIGHT
    # Left 	[3] +
    # Right [3] -
    # Front	[4] +
    # Back	[4] -

    left_x = -joy_message.axes[0]
    left_y = joy_message.axes[1]
    #  right_x = -joy_message.axes[3]
    right_y = joy_message.axes[4]

    teleoperation = Teleoperation()
    teleoperation.speed = set_speed(right_y)
    teleoperation.steering = set_steering(left_x, left_y)

    pub.publish(teleoperation)


def set_speed(y):
    """
    Set speed [-100, +100] from right joystick values [x, y]

    :param float y: Y axe joystick value [-1, 1]
    """

    old_min = -1
    old_max = 1

    # Normalize speed value
    speed = normalize_values_100(y, old_min, old_max)

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
    polar, theta = cartesian2polar(x, y)

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

    sub = rospy.Subscriber("/joy", Joy, callback)
    pub = rospy.Publisher("/teleop_topic", Teleoperation, queue_size=1)

    rospy.spin()
