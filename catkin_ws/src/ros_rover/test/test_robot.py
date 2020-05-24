#!/usr/bin/env python
import sys
import unittest

from scripts.robot import *


class TestRobot(unittest.TestCase):
    def setUp(self):
        # This runs before the test cases are executed
        self.robot = Robot()

    def test_deg_to_tick_lower(self):
        value1 = self.robot.deg_to_tick(-100, 250, 750)
        value2 = 250
        self.assertEquals(value1, value2, str(value1) + " != " + str(value2))

    def test_deg_to_tick_greater(self):
        value1 = self.robot.deg_to_tick(100, 250, 750)
        value2 = 750
        self.assertEquals(value1, value2, str(value1) + " != " + str(value2))

    def test_calculate_target_deg(self):
        value1 = self.robot.calculate_target_deg(200)
        value2 = [0, 0, 0, 0]
        self.assertEquals(value1, value2, str(value1) + " != " + str(value2))

    def test_generate_commands_zero_speed(self):
        value1 = self.robot.generate_commands(0, 0)
        value2 = ([0, 0, 0, 0, 0, 0], [500, 500, 500, 500])
        self.assertEquals(value1, value2, str(value1) + " != " + str(value2))

    def test_generate_commands_zero_radius(self):
        value1 = self.robot.generate_commands(100, 0)
        value2 = ([400, 400, 400, -400, -400, -400], [500, 500, 500, 500])
        self.assertEquals(value1, value2, str(value1) + " != " + str(value2))

    def test_generate_commands_positive(self):
        value1 = self.robot.generate_commands(50, 25)
        value2 = ([196, 200, 196, -156, -152, -156], [535, 540, 470, 465])
        self.assertEquals(value1, value2, str(value1) + " != " + str(value2))

    def test_generate_commands_negative(self):
        value1 = self.robot.generate_commands(-50, -100)
        value2 = ([-100, -68, -96, 200, 192, 196], [295, 400, 690, 590])
        self.assertEquals(value1, value2, str(value1) + " != " + str(value2))

    def test_generate_commands_pos_neg(self):
        value1 = self.robot.generate_commands(50, -25)
        value2 = ([156, 152, 156, -196, -200, -196], [460, 465, 535, 530])
        self.assertEquals(value1, value2, str(value1) + " != " + str(value2))

    def test_generate_commands_neg_pos(self):
        value1 = self.robot.generate_commands(-50, 100)
        value2 = ([-200, -192, -196, 100, 68, 96], [600, 705, 410, 310])
        self.assertEquals(value1, value2, str(value1) + " != " + str(value2))


def suite():
    # Test suite
    suite = unittest.TestSuite()
    suite.addTests(
        unittest.TestLoader().loadTestsFromTestCase(TestRobot)
    )

    return suite


if __name__ == '__main__':
    unittest.TextTestRunner(verbosity=2).run(suite())
