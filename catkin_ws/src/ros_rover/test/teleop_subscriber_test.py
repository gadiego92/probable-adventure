#!/usr/bin/env python

# Publisher/subscriber teleop_topic topic validation

import time
import unittest

import rospy
import rostest

from ros_rover.msg import Teleoperation

PKG = "ros_rover"
NAME = "teleop_subscriber_test"


class TestTeleopTopic(unittest.TestCase):
    def __init__(self, *args):
        super(TestTeleopTopic, self).__init__(*args)
        self.success = False

    def callback(self, data):
        speed = data.speed == 25
        steering = data.steering = 75

        self.success = speed and steering

    def test_teleop_topic(self):
        rospy.init_node(NAME)
        rospy.Subscriber("teleop_topic", Teleoperation, self.callback)
        # 3 seconds
        timeout_t = time.time() + 3.0

        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            time.sleep(0.1)

        self.assertTrue(self.success)


if __name__ == "__main__":
    rostest.rosrun(PKG, NAME, TestTeleopTopic)
