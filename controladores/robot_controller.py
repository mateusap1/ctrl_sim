import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from typing import List

import time
import math


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__("robot_controller")
        timer_period = 1.0  # seconds

        self.velocity_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, "/odom", self.odom_listener_callback, 10
        )

        # We will have the following states:
        #   * adjust_angle
        #   * move_straight
        #   * terminal

        # Our robot should adjust the angle until he gets the right
        # postion. Then he should change the state and move straight
        self.state = "adjust_angle"

        self.lkp = 1
        self.lki = 0.1
        self.linear_errors = []

        self.akp = 1
        self.aki = 0.1
        self.angular_errors = []

        # Where is my end goal?
        self.setpoint = (3.0, 3.0)

        # This should constantly be updated by the odometry
        self.current_position = (0.0, 0.0)
        self.current_angle = 0.0

        self.last_timestamp = -1.0

    def load_odometry(self, msg: Odometry):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.current_position = (position.x, position.y)
        self.current_angle = orientation.z

    def calculate_errors(self):
        sx, sy = self.setpoint
        cx, cy = self.current_position
        delta_x = sx - cx
        delta_y = sy - cy

        expected_angle = math.atan(delta_y / delta_x)

        return (expected_angle - self.current_angle, math.sqrt(delta_x**2 + delta_y**2))

    def adjust_angle_state(self, msg: Odometry):
        self.load_odometry(msg)

        print("_________________________________________")

        print("Odom", self.current_position, self.current_angle)

        aerror, _ = self.calculate_errors()
        angular = self.akp * aerror + self.aki * sum(self.angular_errors)

        print("angular", aerror, angular)

        if (aerror**2) < 0.1:
            self.state = "move_straight"

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = angular

        self.velocity_publisher.publish(msg)
        self.get_logger().info(f"Publishing {msg}")

        print("_________________________________________")

    def move_straight_state(self, msg: Odometry):
        self.load_odometry(msg)

        print("_________________________________________")

        print("Odom", self.current_position, self.current_angle)

        aerror, lerror = self.calculate_errors()
        linear = self.lkp * lerror + self.lki * sum(self.linear_errors)

        print("linear", lerror, linear)

        if (lerror**2) < 0.5:
            self.state = "terminal"
        elif (aerror**2) > 0.2:
            self.state = "adjust_angle"

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = 0.0

        self.velocity_publisher.publish(msg)
        self.get_logger().info(f"Publishing {msg}")

        print("_________________________________________")

    def odom_listener_callback(self, msg: Odometry):
        if self.state == "adjust_angle":
            self.adjust_angle_state(msg)
        elif self.state == "move_straight":
            self.move_straight_state(msg)
        elif self.state == "terminal":
            self.load_odometry(msg)
        else:
            raise ValueError(f"State does not exist {self.state}")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
