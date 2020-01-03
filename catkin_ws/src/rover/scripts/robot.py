#!/usr/bin/env python

import math
import time

import rospy


class Robot():
    '''
    Robot class contains all the math and motor control algorithms to move the rover
    In order to call command the robot the only method necessary is the sendCommands() method with drive velocity and turning amount
    '''

    def __init__(self):
        # distances = rospy.get_param('mech_dist', '7.254,10.5,10.5,10.073').split(",")
        # distances = rospy.get_param( 'mech_dist', '23, 25.5, 28.5, 26' ).split( "," )
        distances = '23, 25.5, 28.5, 26'.split(",")
        self.d1 = float(distances[0])
        self.d2 = float(distances[1])
        self.d3 = float(distances[2])
        self.d4 = float(distances[3])

        # to change when getting Mc thread lock to work
        # enc_min_raw = rospy.get_param( 'enc_min', 0 )
        # enc_max_raw = rospy.get_param( 'enc_max', 1000 )
        enc_min_raw = 0
        enc_max_raw = 1000

        enc_min_int = int(enc_min_raw)
        enc_max_int = int(enc_max_raw)

        self.enc_min = enc_min_int
        self.enc_max = enc_max_int
        self.mids = (self.enc_max + self.enc_min) / 2

    @staticmethod
    def deg2tick(deg, e_min, e_max):
        '''
        Converts a degrees to tick value

        :param int deg  : Degrees value desired
        :param int e_min: The minimum encoder value based on physical stop
        :param int e_max: The maximum encoder value based on physical stop
        '''

        temp = (e_max + e_min) / 2 + ((e_max - e_min) / 90) * deg

        if temp < e_min:
            temp = e_min
        elif temp > e_max:
            temp = e_max

        print(temp, e_max, e_min, deg)

        return temp

    def calculateVelocity(self, v, r):
        '''
        Returns a list of speeds for each individual drive motor based on current turning radius

        :param int v: Drive speed command range from -100 to 100
        :param int r: Current turning radius range from -100 to 100
        '''

        if (v == 0):
            return [0] * 6

        if (abs(r) <= 5):
            # No turning radius, all wheels same speed
            return [v] * 6
        else:
            # Get radius in centimeters (555 - 55)
            radius = 555 - ((500 * abs(r)) / 100.0)

            a = math.pow(self.d2, 2)  # Back - D2
            b = math.pow(self.d3, 2)  # Front - D3

            c = math.pow(radius + self.d1, 2)  # Front/Back - Farthest
            d = math.pow(radius - self.d1, 2)  # Front/Back - Closest

            e = radius - self.d4  # Center - Closest
            f = radius + self.d4  # Center - Farthest

            if (radius < 111):
                # Front - Farthest wheel is the Farthest
                rx = math.sqrt(b + c)
            else:
                # Center - Farthest wheel is the Farthest
                rx = f

            # Get speed of each wheel
            v1 = int(v * (math.sqrt(b + c)) / rx)
            v2 = int(v * (f / rx))
            v3 = int(v * (math.sqrt(a + c)) / rx)
            v4 = int((v * math.sqrt(b + d)) / rx)
            v5 = int(v * (e / rx))
            v6 = int((v * math.sqrt(a + d)) / rx)

            if (r < 0):
                # Turn Left
                velocity = [v4, v5, v6, v1, v2, v3]
            elif (r > 0):
                # Turn Right
                velocity = [v1, v2, v3, v4, v5, v6]

            return velocity

    def calculateTargetDeg(self, radius):
        '''
        Takes a turning radius and calculates what angle [degrees] each corner should be at

        :param int radius: Radius drive command, ranges from -100 (turning left) to 100 (turning right)
        '''

        # Scaled from 555 to 55 centimeters
        if radius == 0:
            r = 555
        elif -100 <= radius <= 100:
            r = 555 - abs(radius) * (555 / 100)
        else:
            r = 555

        if r == 555:
            return [0] * 4

        # Turn Right - Turn Left
        # Front Left - Front Right
        ang1 = int(math.degrees(math.atan(self.d3 / (abs(r) + self.d1))))
        # Front Right - Front Left
        ang2 = int(math.degrees(math.atan(self.d3 / (abs(r) - self.d1))))
        # Back Left - Back Right
        ang3 = int(math.degrees(math.atan(self.d2 / (abs(r) + self.d1))))
        # Back Right - Back Left
        ang4 = int(math.degrees(math.atan(self.d2 / (abs(r) - self.d1))))

        angles = [ang1, ang2, ang3, ang4]
        print(angles)
        # rospy.loginfo( angles )

        if radius > 0:
            # Turn Right
            print(ang1, ang2, -ang3, -ang4)
            return [ang1, ang2, -ang3, -ang4]
        else:
            # Turn Left
            print(-ang2, -ang1, ang4, ang3)
            return [-ang2, -ang1, ang4, ang3]

    def calculateTargetTick(self, tar_enc, cur_enc):
        '''
        Takes the target angle and gets what encoder tick that value is for position control

        :param list [int] tar_enc: List of target angles in degrees for each corner
        '''

        tick = []

        for i in range(4):
            tick.append(self.deg2tick(tar_enc[i], self.enc_min, self.enc_max))

        print(tick)
        # rospy.loginfo(tick)

        return tick

    def generateCommands(self, v, r, encs):
        '''
        Driving method for the Rover, rover will not do any commands if any motor controller
        throws an error

        :param int v: driving velocity command, % based from -100 (backward) to 100 (forward)
        :param int r: driving turning radius command, % based from -100 (left) to 100 (right)
        '''

        # Get speed of each wheel
        speed = self.calculateVelocity(v, r)
        # Get turn of each wheel measured in ticks
        ticks = self.calculateTargetTick(self.calculateTargetDeg(r), encs)

        return (speed, ticks)
