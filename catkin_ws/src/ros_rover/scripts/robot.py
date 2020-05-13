#!/usr/bin/env python

"""
Robot class.
"""

import math

import rospy

# Speed [-100, +100] * 4 = [-400, +400]
SPEED_FACTOR = 4
# Radius from 255 (0 degrees) to 55 centimeters (45 degrees)
MAX_RADIUS = 255
MIN_RADIUS = 55

class Robot(object):
    """
    Robot class contains all the math and motor control algorithms to move the rover

    In order to call command the robot the only method necessary is the sendCommands()
    method with drive velocity and turning amount
    """

    def __init__(self):
        distances = rospy.get_param("hardware_distances", "23,25.5,28.5,26").split(",")
        self.d1 = float(distances[0])
        self.d2 = float(distances[1])
        self.d3 = float(distances[2])
        self.d4 = float(distances[3])

        enc_min_raw = rospy.get_param("enc_min", 250)
        enc_max_raw = rospy.get_param("enc_max", 750)

        enc_min_int = int(enc_min_raw)
        enc_max_int = int(enc_max_raw)

        self.enc_min = enc_min_int
        self.enc_max = enc_max_int
        self.mids = (self.enc_max + self.enc_min) / 2

    @staticmethod
    def deg_to_tick(deg, e_min, e_max):
        """
        Converts a degrees to tick value

        :param int deg  : Degrees value desired
        :param int e_min: The minimum encoder value based on physical stop
        :param int e_max: The maximum encoder value based on physical stop
        """

        temp = (e_max + e_min) / 2 + ((e_max - e_min) / 90) * deg

        if temp < e_min:
            temp = e_min
        elif temp > e_max:
            temp = e_max

        print(temp, e_max, e_min, deg)

        return temp

    def calculate_velocity(self, v, r):
        """
        Returns a list of speeds for each individual drive motor based on current turning radius

        :param int v: Drive speed command range from -100 to 100
        :param int r: Current turning radius range from -100 to 100
        """

        if v == 0:
            return [0] * 6

        if abs(r) <= 5:
            # No turning radius, all wheels same speed
            if r < 0:
                # Go back
                velocity = [-v, -v, -v, v, v, v]
            else:
                # Go ahead
                velocity = [v, v, v, -v, -v, -v]
        else:
            # Get radius in centimeters (MAX_RADIUS (255) to MIN_RADIUS (55))
            radius = MAX_RADIUS - (((MAX_RADIUS - MIN_RADIUS) * abs(r)) / 100.0)

            a = math.pow(self.d2, 2)  # Back - D2
            b = math.pow(self.d3, 2)  # Front - D3

            c = math.pow(radius + self.d1, 2)  # Front/Back - Farthest
            d = math.pow(radius - self.d1, 2)  # Front/Back - Closest

            e = radius - self.d4  # Center - Closest
            f = radius + self.d4  # Center - Farthest

            if radius < 111:
                # Front - Farthest wheel is the Farthest
                rx = math.sqrt(b + c)
            else:
                # Center - Farthest wheel is the Farthest
                rx = f

            # Get speed of each wheel
            abs_v1 = int(abs(v) * (math.sqrt(b + c)) / rx)
            abs_v2 = int(abs(v) * (f / rx))
            abs_v3 = int(abs(v) * (math.sqrt(a + c)) / rx)
            abs_v4 = int((abs(v) * math.sqrt(b + d)) / rx)
            abs_v5 = int(abs(v) * (e / rx))
            abs_v6 = int((abs(v) * math.sqrt(a + d)) / rx)

            if v < 0:
                # Go back
                if r < 0:
                    # Turn Left
                    velocity = [-abs_v4, -abs_v5, -abs_v6, abs_v1, abs_v2, abs_v3]
                else:
                    # Turn Right
                    velocity = [-abs_v1, -abs_v2, -abs_v3, abs_v4, abs_v5, abs_v6]
            else:
                # Go ahead
                if r < 0:
                    # Turn Left
                    velocity = [abs_v4, abs_v5, abs_v6, -abs_v1, -abs_v2, -abs_v3]
                else:
                    # Turn Right
                    velocity = [abs_v1, abs_v2, abs_v3, -abs_v4, -abs_v5, -abs_v6]

        speed = [SPEED_FACTOR * i for i in velocity]

        # Set the speeds between the range [-max_speed, +max_speed]
        return speed

    def calculate_target_deg(self, radius):
        """
        Takes a turning radius and calculates what angle [degrees] each corner should be at

        :param int radius: Radius drive command, ranges from -100 (left) to +100 (right)
        """

        # Scaled from MAX_RADIUS (255) to MIN_RADIUS (55) centimeters
        if radius == 0:
            r = MAX_RADIUS
        elif -100 <= radius <= 100:
            r = MAX_RADIUS - abs(radius) * int(MAX_RADIUS / 100)
        else:
            r = MAX_RADIUS

        if r == MAX_RADIUS:
            return [0] * 4

        # Turn Right - Turn Left
        # Front Left - Front Right
        ang7 = int(math.degrees(math.atan(self.d3 / (abs(r) + self.d1))))
        # Front Right - Front Left
        ang8 = int(math.degrees(math.atan(self.d3 / (abs(r) - self.d1))))
        # Back Left - Back Right
        ang9 = int(math.degrees(math.atan(self.d2 / (abs(r) + self.d1))))
        # Back Right - Back Left
        ang10 = int(math.degrees(math.atan(self.d2 / (abs(r) - self.d1))))

        if radius < 0:
            # Turn Left
            print(-ang8, -ang7, ang10, ang9)
            angles = [-ang8, -ang7, ang10, ang9]
        else:
            # Turn Right
            print(ang7, ang8, -ang9, -ang10)
            angles = [ang7, ang8, -ang9, -ang10]

        return angles

    def calculate_target_tick(self, tar_enc):
        """
        Takes the target angle and gets what encoder tick that value is for position control

        :param list [int] tar_enc: List of target angles in degrees for each corner
        """

        tick = []

        for i in range(4):
            tick.append(self.deg_to_tick(tar_enc[i], self.enc_min, self.enc_max))

        print tick

        return tick

    def generate_commands(self, v, r):
        """
        Driving method for the Rover, rover will not do any commands if any motor controller
        throws an error

        :param int v: driving velocity command, % based from -100 (backward) to 100 (forward)
        :param int r: driving turning radius command, % based from -100 (left) to 100 (right)
        """

        # Get speed of each wheel
        speed = self.calculate_velocity(v, r)
        # Get turn of each wheel measured in ticks
        ticks = self.calculate_target_tick(self.calculate_target_deg(r))

        return (speed, ticks)
