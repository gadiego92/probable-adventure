#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3


def talker():
    pub = rospy.Publisher('chatter', Vector3, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        vector3 = Vector3()
        vector3.x = 1.0
        vector3.y = 2.0
        vector3.z = 3.0
        rospy.loginfo(vector3)
        pub.publish(vector3)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
