import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from typing import Tuple

import time
import math


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__("robot_controller")
        self.velocity_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, "/odom", self.odom_listener_callback, 10
        )
        self.scan_subscriber = self.create_subscription(
            LaserScan, "/scan", self.scan_listener_callback, 10
        )

        self.timer = self.create_timer(1, self.timer_callback)

        self.timer = self.create_timer(0.5, self.timer_callback)

        # Our robot should adjust the angle until he gets the right
        # postion. Then he should change the state and move straight
        self.state = "adjust_angle"

        self.lkp = 1
        self.lki = 0.1
        self.linear_errors = []

        self.akp = 1
        self.aki = 0.01
        self.angular_errors = []

        # Where is my end goal?
        self.setpoint = (-3.0, -3.0)

        # This should constantly be updated by the odometry
        self.current_position = (0.0, 0.0)
        self.current_angle = 0.0

        self.last_timestamp = 0.0

    def load_odometry(self, msg: Odometry):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.current_position = (position.x, position.y)

        self.current_angle = math.atan2(
            2 * orientation.w * orientation.z, 1 - 2 * orientation.z**2
        )

    def odom_listener_callback(self, msg: Odometry):
        self.load_odometry(msg)

    def calculate_errors(self):
        sx, sy = self.setpoint
        cx, cy = self.current_position
        delta_x = sx - cx
        delta_y = sy - cy

        expected_angle = MinimalPublisher.expected_angle(
            self.current_position, self.setpoint
        )

        print("expected angle", expected_angle)
        print("current angle", self.current_angle)

        current_angle = self.current_angle
        if expected_angle < 0:
            expected_angle += 2 * math.pi
        if current_angle < 0:
            current_angle += 2 * math.pi

    def is_near_object(self) -> bool:
        return self.closest_distance <= self.distance_limit

    def adjust_angle_state(self):
        print("_________________________________________")

        print("Odom", self.current_position, self.current_angle)

        aerror, _ = self.calculate_errors()
        angular = self.akp * aerror + self.aki * sum(self.angular_errors)

        current_timestamp = time.time()

        if self.last_timestamp != 0.0:
            self.angular_errors.append(
                aerror * (current_timestamp - self.last_timestamp)
            )

        self.last_timestamp = current_timestamp

        print("angular", aerror, angular)

        if abs(aerror) < 0.1:
            self.state = "move_straight"
            self.angular_errors = []

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = angular

        self.velocity_publisher.publish(msg)
        self.get_logger().info(f"Publishing {msg}")

        print("_________________________________________")

    def move_straight_state(self):
        print("_________________________________________")

        print("Odom", self.current_position, self.current_angle)

        aerror, lerror = self.calculate_errors()
        linear = self.lkp * lerror + self.lki * sum(self.linear_errors)

        print("linear", lerror, linear)
        print("aerror", aerror)

        if abs(lerror) < 0.2:
            self.state = "terminal"
        elif abs(aerror) > 0.2:
            self.state = "adjust_angle"

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = 0.0

        self.velocity_publisher.publish(msg)
        self.get_logger().info(f"Publishing {msg}")

        print("_________________________________________")

    def timer_callback(self):
        print(self.state)
        if self.state == "adjust_angle":
            self.adjust_angle_state()
        elif self.state == "move_straight":
            self.move_straight_state()
        elif self.state == "terminal":
            pass
        else:
            raise ValueError(f"State does not exist {self.state}")

    @staticmethod
    def expected_angle(current_position: Tuple[int, int], setpoint: Tuple[int, int]):
        return math.atan2(
            setpoint[1] - current_position[1], setpoint[0] - current_position[0]
        )


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
