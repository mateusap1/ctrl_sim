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

        self.object_position = (0.0, 0.0)
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
        clear_ranges = MinimalPublisher.front_clear_range(
            msg.angle_min, msg.angle_increment, msg.ranges, 1.0
        )

        self.front_is_clear = MinimalPublisher.is_front_clear(
            clear_ranges, -math.pi / 4, math.pi / 4
        )

        print("clear ranges", clear_ranges)

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
        clear_ranges: List[Tuple[float, float]], min_angle: float, max_angle: float
    ) -> bool:
        for clear_range in clear_ranges:
            if min_angle >= clear_range[0] and max_angle <= clear_range[1]:
                return True

        return False

    @staticmethod
    def front_clear_range(
        angle_min: float,
        angle_incr: float,
        distances: List[float],
        distance_limit: float,
    ) -> List[Tuple[float, float]]:
        angle_ranges: List[Tuple[float, float]] = []

        min_angle_clear = None
        max_angle_clear = None

        for i, distance in enumerate(distances):
            angle = angle_min + i * angle_incr

            if distance <= distance_limit:
                if min_angle_clear is not None and max_angle_clear is not None:
                    angle_ranges.append((min_angle_clear, max_angle_clear))

                min_angle_clear = None
                max_angle_clear = None
            else:
                if min_angle_clear is None:
                    min_angle_clear = angle

                max_angle_clear = angle

        if min_angle_clear is not None and max_angle_clear is not None:
            angle_ranges.append((min_angle_clear, max_angle_clear))

        return angle_ranges


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
