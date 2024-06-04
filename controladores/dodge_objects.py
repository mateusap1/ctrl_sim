import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from typing import List, Tuple

import math


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__("minimal_publisher")
        timer_period = 1.0  # seconds

        self.velocity_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.scan_subscriber = self.create_subscription(
            LaserScan, "/scan", self.scan_listener_callback, 10
        )
        self.odom_subscriber = self.create_subscription(
            Odometry, "/odom", self.odom_listener_callback, 10
        )

        self.robot_position = (0.0, 0.0)
        self.robot_angle = 0.0

        self.front_is_clear = True

        self.i = 0

    def timer_callback(self):
        self.get_logger().info(
            f"Robot Position: {self.robot_position} | {self.robot_angle}"
        )

        if self.front_is_clear:
            msg = Twist()
            msg.linear.x = 1.0
        else:
            msg = Twist()
            msg.angular.z = 0.5

        self.velocity_publisher.publish(msg)
        self.get_logger().info(f"Publishing {msg}")

        self.i += 1

    def scan_listener_callback(self, msg: LaserScan):
        self.front_is_clear = MinimalPublisher.is_front_clear(
            msg.angle_increment, msg.ranges, 1.0, math.pi / 4
        )

        relative_angle = None
        relative_distance = float("inf")
        for i, current_distance in enumerate(msg.ranges):
            current_angle = msg.angle_min + i * msg.angle_increment
            if current_distance < relative_distance:
                relative_distance = current_distance
                relative_angle = current_angle

        if relative_distance == float("inf"):
            return

        robot_position_x, robot_position_y = self.robot_position

        angle = self.robot_angle + relative_angle

        relative_distance_x = math.cos(angle) * relative_distance
        relative_distance_y = math.sin(angle) * relative_distance

        object_position_x = robot_position_x + relative_distance_x
        object_position_y = robot_position_y + relative_distance_y

        self.object_position = (object_position_x, object_position_y)

    def odom_listener_callback(self, msg):
        position = msg.pose.pose.position
        pos_x, pos_y = (position.x, position.y)

        orientation = msg.pose.pose.orientation
        or_z, or_w = (orientation.z, orientation.w)

        self.robot_position = (pos_x, pos_y)
        self.robot_angle = math.atan2(2 * or_w * or_z, 1 - 2 * or_z**2)

    @staticmethod
    def is_front_clear(
        angle_incr: float,
        distances: List[float],
        distance_limit: float,
        breadth: float,
    ) -> bool:
        for i, distance in enumerate(distances):
            angle = MinimalPublisher.normalize_angle((math.pi / 2) - i * angle_incr)
            if angle < -breadth:
                continue
            if angle > breadth:
                continue

            if distance <= distance_limit:
                return False

        return True

    @staticmethod
    def normalize_angle(angle: float):
        if angle >= -math.pi and angle <= math.pi:
            return angle

        if angle > 2 * math.pi:
            return MinimalPublisher.normalize_angle(2 * math.pi % angle)

        if angle < -2 * math.pi:
            return MinimalPublisher.normalize_angle(-(2 * math.pi % angle))

        if angle > math.pi:
            return MinimalPublisher.normalize_angle(angle - 2 * math.pi)

        if angle < math.pi:
            return MinimalPublisher.normalize_angle(angle + 2 * math.pi)


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
