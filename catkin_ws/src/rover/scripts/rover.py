#!/usr/bin/env python
import rospy

from robot import Robot
from rover_msgs.msg import Commands, Teleoperation

global robot
robot = Robot()


def joy_callback(teleoperation_message):
    """
    Callback function called when a Teleoperation message is received from /teleop_topic topic

    :param list teleoperation_message: List with Teleoperation parameters
    """

    command = Commands()
    # Generate commands
    out_cmds = robot.generateCommands(teleoperation_message.vel, teleoperation_message.steering)
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
    joy_sub = rospy.Subscriber("/teleop_topic", Teleoperation, joy_callback)

    rate = rospy.Rate(10)

    rate.sleep()
    rospy.spin()
