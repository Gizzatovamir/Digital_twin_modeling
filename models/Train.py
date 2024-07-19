import math
import datetime
from models.BasicModel import BasicModel
from models.Subscrber import DataSubscriber
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point
from rclpy.node import Node
from collections import deque
from utils.defs import State
from utils.defs import state_string_dict

EPS = 3
R = 10.0


class Train(
    Node,
    BasicModel,
):
    def __init__(self, name: str, topic_name: str, max_len: int = 5, **kwargs):
        super().__init__(name)
        self.name: str = name
        self.children_models = {}
        self.topic_name = topic_name if topic_name else f"/point_stamped" + name
        self.subscription = self.create_subscription(
            PointStamped,
            self.topic_name,
            self.point_callback,
            10,
        )
        # print(f'train topic - {self.topic_name}')
        self.msg_deque: deque = deque(maxlen=max_len)
        self.state: State.UNDEF = State.UNDEF
        self.idle: bool = kwargs.get("idle", False)

    def point_callback(self, msg: PointStamped):
        self.msg_deque.append(msg)
        # print(f"I heard about pose: {msg.header.stamp}, {msg.point}")
        # print(
        #     f"State after msg - {msg.header.stamp} is {bool(self.update())}, coordinates - {msg.point.x, msg.point.y, msg.point.z}"
        # )
        # self.update_state()
        self.state = self.update()

    def update_state(self):
        self.state = self.update()

    def update(self) -> State:
        def check_msg(msg) -> bool:
            # check if train is on the circle
            cur_time = self.get_clock().now().to_msg()
            current_timestamp = cur_time.sec + (cur_time.nanosec / 1e9)
            msg_timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
            return current_timestamp - msg_timestamp < EPS

        res: bool = True
        if len(self.msg_deque) < 5:
            return State.UNDEF
        for msg in self.msg_deque:
            res *= check_msg(msg)
        return State.VALID if res else State.INVALID


if __name__ == "__main__":

    rclpy.init()
    train = Train(0)
    print("start listening")
    rclpy.spin(train)
    train.destroy_node()
    rclpy.shutdown()
