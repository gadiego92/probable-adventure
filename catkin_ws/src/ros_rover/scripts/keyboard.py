#!/usr/bin/env python

import os
import rospy
import select
import sys
import termios
import tty
from geometry_msgs.msg import Vector3

# Speed values
SPEED_STEP_SIZE = 100.0
SPEED_MIN = 0.0
SPEED_MAX = 1000.0
# Angle values
ANGLE_CONSTANT = 0.24
ANGLE_STEP_SIZE = 10.0
ANGLE_MIN = 0.0
ANGLE_HALF = 90.0
ANGLE_MAX = 180.0

msg = """
Rover Control Panel!
------------------------------------------------------
Moving around:
    q (max front speed) w (forward) e (align)
    a (left)            s (stop)    d (right)
    z (max back speed)  x (back)    c (max right)   v (max right)

w/x : increase/decrease speed ( ~ 100 )
a/d : increase/decrease angle (~ 10 grades )

space key, s : force stop

CTRL-C to quit
"""

e = "Communications Failed"


def getKey():
    """ Return key pressed """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def movement(target_speed, target_angle, target_angle_tmp):
    """ Return a string with the current speed and angle (degrees and value) """
    return "currently:\tspeed %s\t angle %s : %s" % (target_speed, target_angle, target_angle_tmp)


def checkSpeedLimit(speed, direction):
    """ Check if the speed is within the limits """
    if abs(speed) < SPEED_MAX:
        if direction:
            speed = speed + 100
        else:
            speed = speed - 100

    return speed


def convertGrades(value):
    """ Convert an angle from degrees (0~180) to a value (0~750) """
    return value / ANGLE_CONSTANT


def checkTurnLimit(angle):
    """ Check if the angle is within the limits """
    if angle < ANGLE_MIN:
        input = ANGLE_MIN
    elif angle > ANGLE_MAX:
        input = ANGLE_MAX
    else:
        input = angle

    return input


if __name__ == "__main__":
    """ Main """
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    # Register client node with the master under rover_teleoperation name
    rospy.init_node('rover_teleoperation')
    # Register rover_teleoperation node as a publisher of rover_motion ROS topic
    pub = rospy.Publisher('rover_motion', Vector3, queue_size=10)

    status = 0
    target_speed = 0.0
    front_target_angle = ANGLE_HALF
    front_target_angle_tmp = ANGLE_HALF / 0.24

    try:
        print(msg)
        while(1):
            key = getKey()
            if key == 'w':
                # Go Forward
                print('W pressed')
                # If target_speed did not have reached the +SPEED_MAX or if it has reached the -SPEED_MAX
                if abs(target_speed) < SPEED_MAX or target_speed == -SPEED_MAX:
                    target_speed = target_speed + SPEED_STEP_SIZE

                status = status + 1
                print(movement(target_speed, front_target_angle, front_target_angle_tmp))
            elif key == 's':
                # Stop
                print('S pressed')
                target_speed = SPEED_MIN

                status = status + 1
                print(movement(target_speed, front_target_angle, front_target_angle_tmp))
            elif key == 'x':
                # Go Back
                print('X pressed')
                # If target_speed did not have reached the -SPEED_MAX or if it has reached the +SPEED_MAX
                if abs(target_speed) < SPEED_MAX or target_speed == SPEED_MAX:
                    target_speed = target_speed - SPEED_STEP_SIZE

                status = status + 1
                print(movement(target_speed, front_target_angle, front_target_angle_tmp))
            elif key == 'a':
                # Turn Left
                print('A pressed')
                if front_target_angle > ANGLE_MIN:
                    front_target_angle = front_target_angle - ANGLE_STEP_SIZE
                    front_target_angle_tmp = convertGrades(front_target_angle)

                status = status + 1
                print(movement(target_speed, front_target_angle, front_target_angle_tmp))
            elif key == 'd':
                # Turn Right
                print('D pressed')
                if front_target_angle < ANGLE_MAX:
                    front_target_angle = front_target_angle + ANGLE_STEP_SIZE
                    front_target_angle_tmp = convertGrades(front_target_angle)

                status = status + 1
                print(movement(target_speed, front_target_angle, front_target_angle_tmp))
            elif key == 'q':
                # Go Forward (+SPEED_MAX)
                print('Q pressed')
                target_speed = SPEED_MAX

                status = status + 1
                print(movement(target_speed, front_target_angle, front_target_angle_tmp))
            elif key == 'e':
                # Align Wheels
                print('E pressed')
                front_target_angle = ANGLE_HALF
                front_target_angle_tmp = convertGrades(front_target_angle)
                status = status + 1
                print(movement(target_speed, front_target_angle, front_target_angle_tmp))
            elif key == 'z':
                # Go Back (-SPEED_MAX)
                print('Z pressed')
                target_speed = -SPEED_MAX

                status = status + 1
                print(movement(target_speed, front_target_angle, front_target_angle_tmp))
            elif key == 'c':
                # Turn Left (ANGLE_MIN)
                print('C pressed')
                front_target_angle = ANGLE_MIN
                front_target_angle_tmp = convertGrades(front_target_angle)
                status = status + 1
                print(movement(target_speed, front_target_angle, front_target_angle_tmp))
            elif key == 'v':
                # Turn Right (ANGLE_MAX)
                print('V pressed')
                front_target_angle = ANGLE_MAX
                front_target_angle_tmp = convertGrades(front_target_angle)
                status = status + 1
                print(movement(target_speed, front_target_angle, front_target_angle_tmp))
            else:
                if (key == '\x03'):
                    # Break infinite while loop
                    print('Control + C pressed')
                    break

            # After 20 keystrokes show the Rover Control Panel
            if status == 20:
                print(msg)
                status = 0

            # Instantiate and publish a ROS Message (Vector3)
            rover_movement = Vector3()
            rover_movement.x = target_speed
            rover_movement.y = front_target_angle_tmp
            pub.publish(rover_movement)

    except:
        """ Exceptions """
        print(e)

    finally:
        """ Finally """
        # Instantiate and publish a ROS Message (Vector3)
        # Stop (SPEED_MIN) and align the wheels (ANGLE_HALF)
        print('Finally')
        rover_movement = Vector3()
        rover_movement.x = SPEED_MIN
        rover_movement.y = convertGrades(checkTurnLimit(ANGLE_HALF))
        pub.publish(rover_movement)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
