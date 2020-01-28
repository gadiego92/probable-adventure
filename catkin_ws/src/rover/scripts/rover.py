#!/usr/bin/env python
import rospy

from robot import Robot
from rover_msgs.msg import Commands, Remote

global robot
robot = Robot()


def joy_callback(remote_message):
    """
    Callback function called when a Remote message is received from /remote_topic topic

    :param list remote_message: List with Remote parameters
    """

    command = Commands()
    # Generate commands
    out_cmds = robot.generateCommands(remote_message.vel, remote_message.steering)
    # Set drive motors values
    command.drive_motor = out_cmds[0]
    # Set corner motors values
    command.corner_motor = out_cmds[1]

    try:
        pub.publish(command)
    except:
        pass


if __name__ == "__main__":
    global pub

    rospy.init_node("rover_node")
    rospy.loginfo("Starting the rover node")

    pub = rospy.Publisher("/commands_topic", Commands, queue_size=1)
    joy_sub = rospy.Subscriber("/remote_topic", Remote, joy_callback)

    rate = rospy.Rate(10)

    rate.sleep()
    rospy.spin()
