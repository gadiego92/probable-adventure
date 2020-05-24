#!/usr/bin/env python

"""
Motor controller node.
"""

import time

import rospy

from ros_rover.msg import Commands
from lewansoul_wrapper import MotorControllers

motor_controller = MotorControllers()
mutex = False


def callback(command_message):
    """
    Callback function called when a Commands message is received from /commands_topic topic

    :param list command_message: A list of corner and motor lists values
    """

    global mutex

    rospy.loginfo(command_message)

    while mutex:
        time.sleep(0.001)
        print "command_message are being buffered"

    mutex = True

    # Send angle values to corner motors
    motor_controller.corner_to_position(command_message.corner_motor)

    # Send speed values to drive motors
    motor_controller.send_motor_duty(command_message.drive_motor)

    mutex = False


def shutdown():
    """
    Stop motors
    """

    print "Killing motors"
    motor_controller.kill_motors()


if __name__ == "__main__":
    rospy.init_node("motor_controller_node")
    rospy.loginfo("Starting the motor_controller node")
    rospy.on_shutdown(shutdown)

    sub = rospy.Subscriber("commands_topic", Commands, callback)

    rate = rospy.Rate(5)

    rospy.spin()
