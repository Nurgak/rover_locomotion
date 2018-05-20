#!/usr/bin/env python

import rospy
import math
import unittest
import rostest
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

PKG = "rover_locomotion"
NAME = "rover_locomotion_test"


class TestRoverLocomotion(unittest.TestCase):
    def __init__(self, *args):
        super(TestRoverLocomotion, self).__init__(*args)

        self.error = rospy.get_param("~error", 0.001)

        # Joint controllers
        self.controllers = rospy.get_param("controllers")

        # Subscribe to command topics
        self.joint_sub = {}
        for controller in self.controllers:
            joint_parameters = rospy.get_param("controllers/%s" % (controller))
            joint_name = joint_parameters["joint"]
            self.joint_sub[joint_name] = rospy.Subscriber(
                "controllers/%s/command" % (controller), Float64,
                self.get_state, joint_name)

        # Publisher for speed command
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # Local copy of the joint commands send by rover_locomotion node
        self.joint_command = {}

        # Wait until the rover_locomotion is ready for publishing (race)
        rospy.sleep(0.1)

    def get_state(self, msg, joint_name):
        # Locally save the joint command
        self.joint_command[joint_name] = msg.data

    def send_command(self, v_linear, v_angular):
        # Send linear and angular speeds to node
        command = Twist()
        command.linear.x = v_linear
        command.angular.z = v_angular
        self.pub.publish(command)

        # Sleep for the node to process and send back all values
        rospy.sleep(0.5)

    def check_command(self, joint_setpoint):
        # Read out the command messages
        for joint_name in joint_setpoint:
            error = math.fabs(joint_setpoint[joint_name] -
                              self.joint_command[joint_name])
            self.assertTrue(error < self.error,
                            "%s, set point = %f, command: %f"
                            % (joint_name, joint_setpoint[joint_name],
                               self.joint_command[joint_name]))

    def test_forward(self):
        self.send_command(1.0, 0.0)

        joint_setpoint = {
            "joint_steer_front_left": 0.0,
            "joint_steer_front_right": 0.0,
            "joint_steer_back_left": 0.0,
            "joint_steer_back_right": 0.0,
            "joint_drive_front_left": 2.0,
            "joint_drive_front_right": 2.0,
            "joint_drive_back_left": 2.0,
            "joint_drive_back_right": 2.0
        }

        self.check_command(joint_setpoint)

    def test_backward(self):
        self.send_command(-1.0, 0.0)

        joint_setpoint = {
            "joint_steer_front_left": 0.0,
            "joint_steer_front_right": 0.0,
            "joint_steer_back_left": 0.0,
            "joint_steer_back_right": 0.0,
            "joint_drive_front_left": -2.0,
            "joint_drive_front_right": -2.0,
            "joint_drive_back_left": -2.0,
            "joint_drive_back_right": -2.0
        }

        self.check_command(joint_setpoint)

    def test_ackermann_forward_left(self):
        self.send_command(1.0, 1.0)

        joint_setpoint = {
            "joint_steer_front_left": 1.570796,
            "joint_steer_front_right": 0.463648,
            "joint_steer_back_left": -1.570796,
            "joint_steer_back_right": -0.463648,
            "joint_drive_front_left": 2.0,
            "joint_drive_front_right": 4.472135,
            "joint_drive_back_left": 2.0,
            "joint_drive_back_right": 4.472135
        }

        self.check_command(joint_setpoint)

    def test_ackermann_forward_right(self):
        self.send_command(1.0, -1.0)

        joint_setpoint = {
            "joint_steer_front_left": -0.463648,
            "joint_steer_front_right": 1.570796,
            "joint_steer_back_left": 0.463648,
            "joint_steer_back_right": -1.570796,
            "joint_drive_front_left": 4.472135,
            "joint_drive_front_right": -2.0,
            "joint_drive_back_left": 4.472135,
            "joint_drive_back_right": -2.0
        }

        self.check_command(joint_setpoint)

    def test_ackermann_backward_left(self):
        self.send_command(-1.0, 1.0)

        joint_setpoint = {
            "joint_steer_front_left": -0.463648,
            "joint_steer_front_right": 1.570796,
            "joint_steer_back_left": 0.463648,
            "joint_steer_back_right": -1.570796,
            "joint_drive_front_left": -4.472136,
            "joint_drive_front_right": 2.0,
            "joint_drive_back_left": -4.472136,
            "joint_drive_back_right": 2.0
        }

        self.check_command(joint_setpoint)

    def test_ackermann_backward_right(self):
        self.send_command(-1.0, -1.0)

        joint_setpoint = {
            "joint_steer_front_left": 1.570796,
            "joint_steer_front_right": 0.463648,
            "joint_steer_back_left": -1.570796,
            "joint_steer_back_right": -0.463648,
            "joint_drive_front_left": -2.0,
            "joint_drive_front_right": -4.472135,
            "joint_drive_back_left": -2.0,
            "joint_drive_back_right": -4.472135
        }

        self.check_command(joint_setpoint)

    def test_point_turn_clockwise(self):
        self.send_command(0.0, 1.0)

        joint_setpoint = {
            "joint_steer_front_left": -0.785398,
            "joint_steer_front_right": 0.785398,
            "joint_steer_back_left": 0.785398,
            "joint_steer_back_right": -0.785398,
            "joint_drive_front_left": -2.828427,
            "joint_drive_front_right": 2.828427,
            "joint_drive_back_left": -2.828427,
            "joint_drive_back_right": 2.828427
        }

        self.check_command(joint_setpoint)

    def test_point_turn_counter_clockwise(self):
        self.send_command(0.0, -1.0)

        joint_setpoint = {
            "joint_steer_front_left": -0.785398,
            "joint_steer_front_right": 0.785398,
            "joint_steer_back_left": 0.785398,
            "joint_steer_back_right": -0.785398,
            "joint_drive_front_left": 2.828427,
            "joint_drive_front_right": -2.828427,
            "joint_drive_back_left": 2.828427,
            "joint_drive_back_right": -2.828427
        }

        self.check_command(joint_setpoint)

    def test_stop(self):
        self.send_command(0.0, 0.0)

        joint_setpoint = {
            "joint_drive_front_left": 0.0,
            "joint_drive_front_right": 0.0,
            "joint_drive_back_left": 0.0,
            "joint_drive_back_right": 0.0
        }

        self.check_command(joint_setpoint)

if __name__ == "__main__":
    rospy.init_node(NAME, anonymous=True)
    rostest.rosrun(PKG, NAME, TestRoverLocomotion)
