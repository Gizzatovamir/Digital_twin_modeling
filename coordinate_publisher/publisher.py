import pathlib
import rclpy
from geometry_msgs.msg import PointStamped, Point
from rclpy.publisher import Publisher
from std_msgs.msg import Header
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
import numpy as np
import os
from typing import Tuple
import math

_debug = True


class DataPublisher(Node):
    def __init__(self, step_size: float, name: str, topic_name: str = ""):
        super().__init__(f"data_publisher_" + name)

        self._publisher: Publisher = self.create_publisher(
            PointStamped, topic_name if topic_name else f"/point_stamped_" + name, 10
        )
        timer_period: float = 0.5
        self.step_size: float = step_size
        self.i: int = 0
        self.a: float = 0
        self.b: float = 0
        self.r: float = 10.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.last_timestamp: int = 0
        self.t: float = 0

    def get_coord(self) -> Point:
        if self.t < 2 * math.pi:
            self.t += self.step_size
        else:
            self.t = 0
        point = Point()
        point.x = self.r * math.cos(self.t) + self.a
        point.y = self.r * math.sin(self.t) + self.b
        point.z = 0.0
        return point

    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.5 seconds.
        """
        msg = PointStamped()
        header: Header = Header()

        # set current timestamp
        header.stamp = self.get_clock().now().to_msg()

        # Set the msg data
        msg.header = header
        msg.point = self.get_coord()

        # Publish the message to the topic
        self._publisher.publish(msg)

        # Display the message on the console
        self.get_logger().info(
            f"Publishing: {self.i}, "
            f"timestamp - {header.stamp}, "
            f"x - {msg.point.x}, "
            f"y - {msg.point.y}"
        )

        # Increment the counter by 1
        self.i += 1


if __name__ == "__main__":
    rclpy.init()
    publisher = DataPublisher(0.1)
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()
