#!/usr/bin/env python

# Simple publisher demo that publishes msg/Teleoperation messages
# to the "teleop_topic" topic

import rospy

from ros_rover.msg import Teleoperation


def talker():
    rospy.init_node("talker")
    # Register talker node as a publisher of teleop_topic ROS topic
    pub = rospy.Publisher("teleop_topic", Teleoperation, queue_size=1)
    rate = rospy.Rate(10)  # 10hz

    # Instantiate and publish a ROS Message (Teleoperation)
    teleoperation = Teleoperation()
    teleoperation.speed = 25
    teleoperation.steering = 75

    while not rospy.is_shutdown():
        pub.publish(teleoperation)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
