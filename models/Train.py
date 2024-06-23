import math

from .BasicModel import BasicModel
from .Subscrber import DataSubscriber
import rclpy
from geometry_msgs.msg import PointStamped, Point
from rclpy.node import Node
from collections import deque

EPS = 1e-6
R = 10.0


class Train(
    Node,
    BasicModel,
):
    def __init__(self, index: int, max_len: int = 5):
        super().__init__("test")
        self.subscription = self.create_subscription(
            PointStamped, f"/point_stamped_{index}", self.point_callback, 10
        )
        self.msg_deque: deque = deque(maxlen=5)

    def point_callback(self, msg: PointStamped):
        self.msg_deque.append(msg)
        self.get_logger().info(f"I heard about pose: {msg.header.stamp}, {msg.point}")
        print(self.update())

    def update(self) -> bool:
        def check_msg(msg) -> bool:
            # check if train is on the circle
            print(abs(math.sqrt(msg.point.x**2 + msg.point.y**2 + msg.point.z**2) - R))
            return (
                abs(math.sqrt(msg.point.x**2 + msg.point.y**2 + msg.point.z**2) - R)
                < EPS
            )

        res: bool = True
        for msg in self.msg_deque:
            res *= check_msg(msg)
        return res


if __name__ == "__main__":

    rclpy.init()
    train = Train(0)
    print("start listening")
    rclpy.spin(train)

    train.destroy_node()
    rclpy.shutdown()
