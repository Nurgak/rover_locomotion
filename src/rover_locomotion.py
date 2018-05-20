#!/usr/bin/env python

import rospy
import math
import tf
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class Locomotion():
    def __init__(self):
        # Get the locomotion joints controllers from the yaml file
        self.controllers = rospy.get_param("controllers")

        # Values are in SI units
        self.wheel_radius = rospy.get_param("wheel_radius")

        # Populate the joint publishers
        self.joint_pub = {}
        for controller in self.controllers:
            joint_parameters = rospy.get_param("controllers/%s" % (controller))
            joint_name = joint_parameters["joint"]
            self.joint_pub[joint_name] = rospy.Publisher(
                "controllers/%s/command" % (controller), Float64,
                queue_size=10)

        # Transform listener for wheel placement
        self.tf_listener = tf.TransformListener()

    def set_speed(self, twist):
        v_linear = twist.linear.x
        v_angular = twist.angular.z

        rospy.logdebug("Setting new speed, linear: %fm/s, angular: %frad/s" %
                       (v_linear, v_angular))

        # Stop the motors
        if v_angular == 0 and v_linear == 0:
            rospy.logdebug("Stopping all drive wheels")
            for joint_name in self.joint_pub:
                if "drive" in joint_name:
                    velocity = Float64()
                    velocity.data = 0.0
                    self.joint_pub[joint_name].publish(velocity)
            # Return here to not set the steering angles (damages couplings)
            return

        # Evaluate the turn radius based on linear and angular velocities
        if v_angular == 0:
            turn_radius = float('inf')
        else:
            turn_radius = v_linear / v_angular

        for joint_name in self.joint_pub:
            if "steer" in joint_name:
                # Get the wheel instance from the name of the joint
                wheel_name = joint_name.replace("joint_steer", "wheel")

                # Find the position of the wheel relative to the body
                try:
                    (translation, rotation) = self.tf_listener.lookupTransform(
                        "base_link", wheel_name, rospy.Time(0))
                except:
                    rospy.logerr("Transform between base_link and %s not found"
                                 % (wheel_name))
                    continue

                # Set the steering angle
                angle = Float64()

                x = translation[0] - rospy.get_param("~turn_offset_x", 0)
                y = turn_radius - translation[1]

                if y == float('inf'):
                    angle.data = 0.0
                elif y == 0.0 and x >= 0:
                    # Infinite y or singularity x means "go straight"
                    angle.data = math.pi / 2
                elif y == 0.0 and x < 0:
                    # Infinite y or singularity x means "go straight"
                    angle.data = -math.pi / 2
                else:
                    angle.data = math.atan(x / y)

                self.joint_pub[joint_name].publish(angle)

            elif "drive" in joint_name:
                # Get the wheel instance from the name of the joint
                wheel_name = joint_name.replace("joint_drive", "wheel")

                # Find the position of the wheel relative to the body
                try:
                    (translation, rotation) = self.tf_listener.lookupTransform(
                        "base_link", wheel_name, rospy.Time(0))
                except:
                    rospy.logerr("Transform between base_link and %s not found"
                                 % (wheel_name))
                    continue

                # Set the drive velocity
                velocity = Float64()

                x = translation[0] - rospy.get_param("~turn_offset_x", 0)
                y = turn_radius - translation[1]

                wheel_turn_radius = math.sqrt(y**2 + x**2)
                delta = v_angular * math.sqrt(y**2 + x**2) / self.wheel_radius

                if y == float('inf'):
                    # Infinite y or null x means "go straight"
                    velocity.data = v_linear / self.wheel_radius
                elif y >= 0:
                    velocity.data = delta
                else:
                    velocity.data = -delta

                self.joint_pub[joint_name].publish(velocity)

    def stop(self):
        speed = Twist()
        self.set_speed(speed)


def setup():
    rospy.init_node("rover_locomotion", log_level=rospy.DEBUG, anonymous=False)

    loc = Locomotion()

    # Relative parameter
    command_topic = rospy.get_param("~command_topic", "/cmd_vel")
    rospy.Subscriber(command_topic, Twist, loc.set_speed, queue_size=1)

    # Stop the robot before stopping the node
    rospy.on_shutdown(loc.stop)

    rospy.spin()

if __name__ == "__main__":
    setup()
