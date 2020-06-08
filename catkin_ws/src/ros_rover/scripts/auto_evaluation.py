#!/usr/bin/env python

"""
Auto Evaluation node.
"""
import rospy

from ros_rover.msg import Teleoperation

# Speed values
SPEED_MIN = -50
SPEED_ZERO = 0
SPEED_MAX = 50
# Angle values
ANGLE_MIN = -100
ANGLE_ZERO = 0
ANGLE_MAX = 100


def main():
    # Loginfo message
    rospy.loginfo("Starting evaluation_node ...")

    # Register client node with the master under evaluation_node name
    rospy.init_node("evaluation_node")

    # Register evaluation_node node as a publisher of teleop_topic ROS topic
    pub = rospy.Publisher("teleop_topic", Teleoperation, queue_size=1)

    # Instantiate and publish a ROS Message (Teleoperation)
    teleoperation = Teleoperation()
    teleoperation_zero = Teleoperation()
    teleoperation_zero.speed = SPEED_ZERO
    teleoperation_zero.steering = ANGLE_ZERO
    # Initialize status
    status = 0

    while 1:
        # Sleep 1 second
        rospy.loginfo("Sleep 2 seconds ...")
        rospy.sleep(2)

        if status == 0:
            # Go Forward
            status = 1

            teleoperation.speed = SPEED_MAX
            teleoperation.steering = ANGLE_ZERO

            rospy.loginfo("Moving forward!")
            for i in range(30):
                pub.publish(teleoperation)
                rospy.sleep(0.1)
        else:
            # Go Back
            status = 0

            teleoperation.speed = SPEED_MIN
            teleoperation.steering = ANGLE_ZERO

            rospy.loginfo("Moving back!")
            for i in range(30):
                pub.publish(teleoperation)
                rospy.sleep(0.1)

        # Stop
        pub.publish(teleoperation_zero)

        # Sleep 1 second
        rospy.loginfo("Sleep 2 seconds ...")
        rospy.sleep(2)

        # Turn wheels
        rospy.loginfo("Turn left!")
        teleoperation.speed = SPEED_ZERO
        teleoperation.steering = ANGLE_MIN
        pub.publish(teleoperation)
        rospy.loginfo("Sleep 2 seconds ...")
        rospy.sleep(2)

        rospy.loginfo("Turn front!")
        teleoperation.speed = SPEED_ZERO
        teleoperation.steering = ANGLE_ZERO
        pub.publish(teleoperation)
        rospy.loginfo("Sleep 2 seconds ...")
        rospy.sleep(2)

        rospy.loginfo("Turn right!")
        teleoperation.speed = SPEED_ZERO
        teleoperation.steering = ANGLE_MAX
        pub.publish(teleoperation)
        rospy.loginfo("Sleep 2 seconds ...")
        rospy.sleep(2)

        rospy.loginfo("Turn front!")
        teleoperation.speed = SPEED_ZERO
        teleoperation.steering = ANGLE_ZERO
        pub.publish(teleoperation)


if __name__ == "__main__":
    main()
