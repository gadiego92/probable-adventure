#!/usr/bin/env python

import time
import unittest

import serial

from scripts.lewansoul_wrapper import MotorControllers

MOTOR_LEFT_FRONT = 1
MOTOR_LEFT_MIDDLE = 2
MOTOR_LEFT_BACK = 3
MOTOR_RIGHT_FRONT = 4
MOTOR_RIGHT_MIDDLE = 5
MOTOR_RIGHT_BACK = 6

SERVO_LEFT_FRONT = 7
SERVO_RIGHT_FRONT = 8
SERVO_LEFT_BACK = 9
SERVO_RIGHT_BACK = 10

NUMBER_0 = 0
NUMBER_500 = 500


class TestLewansoul(unittest.TestCase):

    def setUp(self):
        # This runs before the test cases are executed
        self.motor_controller = MotorControllers()

    def test_servo_get_motor_speed(self):
        servo_left_front = self.motor_controller.get_motor_speed(
            SERVO_LEFT_FRONT)
        self.assertFalse(servo_left_front)

    def test_kill_motors(self):
        self.motor_controller.kill_motors()

        motor_left_front = self.motor_controller.get_motor_speed(
            MOTOR_LEFT_FRONT)
        motor_left_middle = self.motor_controller.get_motor_speed(
            MOTOR_LEFT_MIDDLE)
        motor_left_back = self.motor_controller.get_motor_speed(
            MOTOR_LEFT_BACK)
        motor_right_front = self.motor_controller.get_motor_speed(
            MOTOR_RIGHT_FRONT)
        motor_right_middle = self.motor_controller.get_motor_speed(
            MOTOR_RIGHT_MIDDLE)
        motor_right_back = self.motor_controller.get_motor_speed(
            MOTOR_RIGHT_BACK)
        servo_left_front = self.motor_controller.get_corner_position(
            SERVO_LEFT_FRONT)
        servo_right_front = self.motor_controller.get_corner_position(
            SERVO_RIGHT_FRONT)
        servo_left_back = self.motor_controller.get_corner_position(
            SERVO_LEFT_BACK)
        servo_right_back = self.motor_controller.get_corner_position(
            SERVO_RIGHT_BACK)

        self.assertEquals(motor_left_front, NUMBER_0, str(
            motor_left_front) + " != " + str(NUMBER_0))
        self.assertEquals(motor_left_middle, NUMBER_0, str(
            motor_left_middle) + " != " + str(NUMBER_0))
        self.assertEquals(motor_left_back, NUMBER_0, str(
            motor_left_back) + " != " + str(NUMBER_0))
        self.assertEquals(motor_right_front, NUMBER_0, str(
            motor_right_front) + " != " + str(NUMBER_0))
        self.assertEquals(motor_right_middle, NUMBER_0, str(
            motor_right_middle) + " != " + str(NUMBER_0))
        self.assertEquals(motor_right_back, NUMBER_0, str(
            motor_right_back) + " != " + str(NUMBER_0))
        self.assertAlmostEqual(servo_left_front, NUMBER_500, delta=2, msg=str(
            servo_left_front) + " != " + str(NUMBER_500))
        self.assertAlmostEqual(servo_right_front, NUMBER_500, delta=2, msg=str(
            servo_right_front) + " != " + str(NUMBER_500))
        self.assertAlmostEqual(servo_left_back, NUMBER_500, delta=2, msg=str(
            servo_left_back) + " != " + str(NUMBER_500))
        self.assertAlmostEqual(servo_right_back, NUMBER_500, delta=2, msg=str(
            servo_right_back) + " != " + str(NUMBER_500))

    def test_corner_to_position(self):
        corner_ticks = [NUMBER_500, NUMBER_500, NUMBER_500, NUMBER_500]

        self.motor_controller.corner_to_position(corner_ticks)

        servo_left_front = self.motor_controller.get_corner_position(
            SERVO_LEFT_FRONT)
        servo_right_front = self.motor_controller.get_corner_position(
            SERVO_RIGHT_FRONT)
        servo_left_back = self.motor_controller.get_corner_position(
            SERVO_LEFT_BACK)
        servo_right_back = self.motor_controller.get_corner_position(
            SERVO_RIGHT_BACK)

        self.assertAlmostEqual(servo_left_front, NUMBER_500, delta=2, msg=str(
            servo_left_front) + " != " + str(NUMBER_500))
        self.assertAlmostEqual(servo_right_front, NUMBER_500, delta=2, msg=str(
            servo_right_front) + " != " + str(NUMBER_500))
        self.assertAlmostEqual(servo_left_back, NUMBER_500, delta=2, msg=str(
            servo_left_back) + " != " + str(NUMBER_500))
        self.assertAlmostEqual(servo_right_back, NUMBER_500, delta=2, msg=str(
            servo_right_back) + " != " + str(NUMBER_500))

    def test_send_motor_duty(self):
        drive_ticks = [NUMBER_0, NUMBER_0,
                       NUMBER_0, NUMBER_0, NUMBER_0, NUMBER_0]

        self.motor_controller.send_motor_duty(drive_ticks)

        motor_left_front = self.motor_controller.get_motor_speed(
            MOTOR_LEFT_FRONT)
        motor_left_middle = self.motor_controller.get_motor_speed(
            MOTOR_LEFT_MIDDLE)
        motor_left_back = self.motor_controller.get_motor_speed(
            MOTOR_LEFT_BACK)
        motor_right_front = self.motor_controller.get_motor_speed(
            MOTOR_RIGHT_FRONT)
        motor_right_middle = self.motor_controller.get_motor_speed(
            MOTOR_RIGHT_MIDDLE)
        motor_right_back = self.motor_controller.get_motor_speed(
            MOTOR_RIGHT_BACK)

        self.assertEquals(motor_left_front, NUMBER_0, str(
            motor_left_front) + " != " + str(NUMBER_0))
        self.assertEquals(motor_left_middle, NUMBER_0, str(
            motor_left_middle) + " != " + str(NUMBER_0))
        self.assertEquals(motor_left_back, NUMBER_0, str(
            motor_left_back) + " != " + str(NUMBER_0))
        self.assertEquals(motor_right_front, NUMBER_0, str(
            motor_right_front) + " != " + str(NUMBER_0))
        self.assertEquals(motor_right_middle, NUMBER_0, str(
            motor_right_middle) + " != " + str(NUMBER_0))
        self.assertEquals(motor_right_back, NUMBER_0, str(
            motor_right_back) + " != " + str(NUMBER_0))

    def test_get_status(self):
        value = self.motor_controller.get_status(MOTOR_LEFT_FRONT)
        self.assertFalse(value)

    def test_get_mode(self):
        value = self.motor_controller.get_mode(MOTOR_LEFT_FRONT)
        self.assertTrue(value)

    def test_move(self):
        self.motor_controller.move(SERVO_LEFT_FRONT, NUMBER_500, NUMBER_0)
        servo_left_front = self.motor_controller.get_corner_position(
            SERVO_LEFT_FRONT)
        self.assertAlmostEqual(servo_left_front, NUMBER_500, delta=2, msg=str(
            servo_left_front) + " != " + str(NUMBER_500))

    def test_leds(self):
        self.motor_controller.led_turn_off(SERVO_LEFT_FRONT)
        # Delays for 1 second
        time.sleep(1)
        self.motor_controller.led_turn_on(SERVO_LEFT_FRONT)


def suite():
    # Test suite
    suite = unittest.TestSuite()
    suite.addTests(
        unittest.TestLoader().loadTestsFromTestCase(TestLewansoul)
    )

    return suite


if __name__ == '__main__':
    unittest.TextTestRunner(verbosity=2).run(suite())
