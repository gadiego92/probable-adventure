#!/usr/bin/env python
import sys
import unittest

from scripts import remote


class TestRemote(unittest.TestCase):

    def test_set_speed(self):
        value1 = remote.set_speed(0, 0)
        value2 = -100
        self.assertEquals(value1, value2, str(value1) + " != " + str(value2))

    def test_set_speed_square(self):
        value1 = remote.set_speed(0, 1)
        value2 = 100
        self.assertEquals(value1, value2, str(value1) + " != " + str(value2))

    def test_set_steering_zero(self):
        value1 = remote.set_steering(0, 0)
        value2 = 0
        self.assertEquals(value1, value2, str(value1) + " != " + str(value2))

    def test_set_steering_theta(self):
        value1 = remote.set_steering(-1, -1)
        value2 = -50
        self.assertEquals(value1, value2, str(value1) + " != " + str(value2))

    def test_set_steering(self):
        value1 = remote.set_steering(1, 1)
        value2 = 50
        self.assertEquals(value1, value2, str(value1) + " != " + str(value2))


def suite():
    # Test suite
    suite = unittest.TestSuite()
    suite.addTests(
        unittest.TestLoader().loadTestsFromTestCase(TestRemote)
    )

    return suite


if __name__ == '__main__':
    unittest.TextTestRunner(verbosity=2).run(suite())
