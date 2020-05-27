# Probable Adeventure

## Structure

The project structure is the following:

- `arduino_rover`: arduino example that runs the Rover.
- `catkin_ws`: catkin workspace that contains the ros_rover package.
- `examples`: arduino examples.
- `rosserial_rover`: rosserial arduino example.

## Compile and run the ROS package

To compile the ROS project run the following commands:

Clone the repository:

```
cd ~/Documents
$ git clone git@github.com:gadiego92/probable-adventure.git
```

Compile the workspace using `catkin_make` command:

```
$ cd ~/Documents/probable-adventure/catkin_ws
$ catkin_make
```

Make sure you source your workspace's `setup.sh` file after calling `catkin_make`:

```
$ source ./devel/setup.bash
```

## Running the test

### Python Unittest

To run the Python Unittest we use `coverage` package:

Add the `ros_rover` path to the `PYTHONPATH` environment variable:

```
$ PYTHONPATH=$PYTHONPATH:/home/diego/Documents/probable-adventure/catkin_ws/src/ros_rover
```

Execute the `roscore` command:

```
$ roscore
```

Run the unit tests using the `coverage` command:

```
$ cd ~/Documents/probable-adventure/catkin_ws
$ coverage run src/ros_rover/test/test_remote.py
```

The output should be similar to this:

```
test_set_speed (__main__.TestRemote) ... ok
test_set_speed_square (__main__.TestRemote) ... ok
test_set_steering (__main__.TestRemote) ... ok
test_set_steering_theta (__main__.TestRemote) ... ok
test_set_steering_zero (__main__.TestRemote) ... ok

----------------------------------------------------------------------
Ran 5 tests in 0.001s

OK
```

You can see the `coverage` of the tests using the following command:

```
$ coverage html
```

This command generates a directory with `html` files where you can see the test's coverage.

### ROS Nodes Unittest

To run the ROS Node tests run the following commands:

```
$ cd ~/Documents/probable-adventure/catkin_ws
$ catkin_make run_tests
```

The output will show the test results:

```
[ROSTEST]-----------------------------------------------------------------------

[ros_rover.rosunit-teleop_subscriber_test/test_teleop_topic][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 1
 * ERRORS: 0
 * FAILURES: 0

```
