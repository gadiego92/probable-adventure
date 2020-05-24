#!/usr/bin/env python

# Simple publisher demo that publishes msg/Commands messages
# to the "commands_topic" topic

import rospy

from ros_rover.msg import Commands


def talker():
    rospy.init_node("talker")
    # Register talker node as a publisher of commands_topic ROS topic
    pub = rospy.Publisher("commands_topic", Commands, queue_size=1)
    rate = rospy.Rate(10)  # 10hz

    # Instantiate and publish a ROS Message (Commands)
    commands = Commands()
    commands.drive_motor = [25, 75]
    commands.corner_motor = [0, 100]

    while not rospy.is_shutdown():
        pub.publish(commands)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
