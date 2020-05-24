#!/usr/bin/env python

# Publisher/subscriber commands_topic topic validation

import time
import unittest

import rospy
import rostest

from ros_rover.msg import Commands

PKG = "ros_rover"
NAME = "commands_subscriber_test"


class TestCommandsTopic(unittest.TestCase):
    def __init__(self, *args):
        super(TestCommandsTopic, self).__init__(*args)
        self.success = False

    def callback(self, data):
        drive_motor = data.drive_motor[0] == 25 and data.drive_motor[1] == 75
        corner_motor = data.corner_motor[0] == 0 and data.corner_motor[1] == 100

        self.success = drive_motor and corner_motor

    def test_commands_topic(self):
        rospy.init_node(NAME)
        rospy.Subscriber("commands_topic", Commands, self.callback)
        # 3 seconds
        timeout_t = time.time() + 3.0

        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            time.sleep(0.1)

        self.assertTrue(self.success)


if __name__ == "__main__":
    rostest.rosrun(PKG, NAME, TestCommandsTopic)
