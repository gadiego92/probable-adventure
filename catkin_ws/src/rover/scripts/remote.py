#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from rover_msgs.msg import Remote

import math


def callback(data):
    """
        Callback function called when a msg is received from /joy topic
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

    left_x = -data.axes[0]
    left_y = data.axes[1]
    right_x = -data.axes[3]
    right_y = data.axes[4]

    joy_out = Remote()
    joy_out.x = set_speed(right_y)
    joy_out.y = set_steering(left_x, left_y)

    pub.publish(joy_out)


def set_speed(y):
    """
        Set speed [-100, +100] from right joystick values [x, y]
    """

    min_point = 0
    max_point = 180
    min_speed = -100
    max_speed = 100

    # Normalize speed value
    speed = normmalize_values(y, min_point, max_point, min_speed, max_speed)

    return speed


def set_steering(x, y):
    """
        Set radius [-100, +100] from Left joystick values [x, y]
    """

    # Convert cartesian coordinates to polar coordinates
    polar, theta = cartesian2polar(x, y)

    # Convert negative angle to corresponding positive
    if theta < 0:
        theta = theta * -1

    min_angle = 0
    max_angle = 180
    min_radius = -100
    max_radius = 100

    # Normalize radius angle value
    radius = normmalize_values(theta, min_angle, max_angle, min_radius, max_radius)

    return radius


def cartesian2polar(x, y):
    """
        Convert cartesian coordinates to polar coordinates
    """

    polar = math.sqrt(math.pow(x, 2) + math.pow(y, 2))

    try:
        # Get angle from 2-argument arctangent
        theta = math.degrees(math.atan2(y, x))
    except:
        # Go front [0 - 180]
        theta = 90

    return polar, theta


def normmalize_values(old_value, old_min, old_max, new_min, new_max):
    """
        Normalize and old_value [old_min, old_max] to a new_value [new_min, new_max]
    """

    # Normalize [-1, 1]
    # range = max(a) - min(a);
    range_angle = old_max - old_min
    # a = (a - min(a)) / range;
    zero_one_value = (old_value - old_min) / range_angle

    # Normalize [-100, 100]
    min_radius = -100
    max_radius = 100
    # range2 = y - x;
    range_radius = min_radius - max_radius
    # a = (a * range2) + x;
    new_value = (zero_one_value * range_radius) + max_radius

    return new_value


if __name__ == "__main__":
    global pub

    rospy.init_node("remote_node")
    rospy.loginfo("Starting the remote node")

    sub = rospy.Subscriber("/joy", Joy, callback)
    pub = rospy.Publisher("/remote_topic", Remote, queue_size=1)

    rospy.spin()
