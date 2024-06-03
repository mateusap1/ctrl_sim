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
        self.setpoint = (3.0, 3.0)

        # This should constantly be updated by the odometry
        self.current_position = (0.0, 0.0)
        self.current_angle = 0.0

        self.distance_limit = 1.0
        self.closest_distance = math.inf
        self.closest_angle = 0.0
        self.left_better = False

        self.last_timestamp = 0.0

    def load_odometry(self, msg: Odometry):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.current_position = (position.y, position.x)
        # self.current_position = (position.x, position.y)
        self.current_angle = orientation.z

    def odom_listener_callback(self, msg: Odometry):
        self.load_odometry(msg)

    def scan_listener_callback(self, msg):
        closest_distance = math.inf
        closest_angle = 0.0
        closest_index = -1
        for i, distance in enumerate(msg.ranges):
            # angle = msg.angle_min + i * msg.angle_increment + (math.pi / 2)
            angle = msg.angle_min + i * msg.angle_increment
            if distance < self.closest_distance and (angle < (math.pi / 2)):
                closest_distance = distance
                closest_angle = angle

        self.closest_distance = closest_distance
        self.closest_angle = closest_angle

        angle_range = int((math.pi / 2) / msg.angle_increment)  # 90 degrees
        left_sum = 0
        right_sum = 0
        for i, distance in enumerate(msg.ranges):
            if i < (closest_index - angle_range):
                continue
            if i > (closest_index + angle_range):
                break

            if distance > self.distance_limit:
                if i < closest_index:
                    left_sum += 1
                else:
                    right_sum += 1

        self.left_better = left_sum > right_sum

    def calculate_errors(self):
        sx, sy = self.setpoint
        cx, cy = self.current_position
        delta_x = sx - cx
        delta_y = sy - cy

        expected_angle = MinimalPublisher.expected_angle(self.current_position, self.setpoint)

        print("expected angle", expected_angle)
        print("current angle", self.current_angle)

        return (expected_angle - self.current_angle, math.sqrt(delta_x**2 + delta_y**2))

    def is_near_object(self) -> bool:
        return self.closest_distance <= self.distance_limit

    def adjust_angle_state(self):
        print("_________________________________________")

        print("Odom", self.current_position, self.current_angle)

        aerror, _ = self.calculate_errors()
        angular = self.akp * aerror + self.aki * sum(self.angular_errors)

        current_timestamp = time.time()

        if self.last_timestamp != 0.0:
            self.angular_errors.append(aerror * (current_timestamp - self.last_timestamp))

        self.last_timestamp = current_timestamp

        print("angular", aerror, angular)

        if self.is_near_object():
            self.state = "dodge_angle"
            self.angular_errors = []
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

        if abs(lerror) < 0.2:
            self.state = "terminal"
        elif self.is_near_object():
            self.state = "dodge_angle"
        elif abs(aerror) > 0.2:
            self.state = "adjust_angle"

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = 0.0

        self.velocity_publisher.publish(msg)
        self.get_logger().info(f"Publishing {msg}")

        print("_________________________________________")

    def dodge_angle_state(self):
        # Makes the robot rotate until it becomes perpendicular
        # to the closest object

        print("closest angle", self.closest_angle)

        angle_goal = math.pi / 2
        if abs(self.closest_angle) >= angle_goal:
            self.state = "dodge_straight"

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.25 if self.left_better else -0.25

        self.velocity_publisher.publish(msg)
        self.get_logger().info(f"Publishing {msg}")

    def dodge_straight_state(self):
        # Makes the robot move straight until it becomes far
        # from the closes object

        print("closest distance", self.closest_distance)

        distance_goal = self.distance_limit * 1.2
        if self.closest_distance > distance_goal:
            self.state = "move_straight"
        elif self.closest_distance <= self.distance_limit:
            self.state = "dodge_angle"

        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.0

        self.velocity_publisher.publish(msg)
        self.get_logger().info(f"Publishing {msg}")

    def timer_callback(self):
        print(self.state)
        if self.state == "adjust_angle":
            self.adjust_angle_state()
        elif self.state == "move_straight":
            self.move_straight_state()
        elif self.state == "dodge_angle":
            self.dodge_angle_state()
        elif self.state == "dodge_straight":
            self.dodge_straight_state()
        elif self.state == "terminal":
            pass
        else:
            raise ValueError(f"State does not exist {self.state}")

    @staticmethod
    def expected_angle(current_position: Tuple[int, int], setpoint: Tuple[int, int]):
        distance_vertical = setpoint[0] - current_position[0]
        distance_horizontal = setpoint[1] - current_position[1]

        delta = None
        if distance_horizontal == 0.0:
            if distance_vertical > 0:
                delta = math.inf
            else:
                delta = -math.inf
        else:
            delta = distance_vertical / distance_horizontal

        result = abs(math.atan(delta))

        if distance_vertical >= 0.0:
            if distance_horizontal >= 0.0:
                # Primeiro quadrante
                return (math.pi / 2) - result
            else:
                # Quarto quadrante
                return -((math.pi / 2) - result)
        else:
            if distance_horizontal >= 0.0:
                # Segundo quadrante
                return (math.pi / 2) + result
            else:
                # Terceiro quadrante
                return -((math.pi / 2) + ((math.pi / 2) - result))


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
