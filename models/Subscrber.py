import rclpy
from rclpy.node import Node
from typing import Union

# from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PointStamped, Point
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
import message_filters
import argparse
import pathlib
import os


class DataSubscriber(Node):
    def __init__(self, name: str, **kwargs):
        super().__init__(name)
        self.subscription = self.create_subscription(
            PointStamped, f"/point_stamped_" + name, self.point_callback, 10
        )

    def point_callback(self, msg: PointStamped):
        self.get_logger().info(f"I heard about pose: {msg.header.stamp}, {msg.point}")


if __name__ == "__main__":
    rclpy.init()
    my_subscriber = DataSubscriber("test")
    print("start listening")
    rclpy.spin(my_subscriber)

    my_subscriber.destroy_node()
    rclpy.shutdown()
