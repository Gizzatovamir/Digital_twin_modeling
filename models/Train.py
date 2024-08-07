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

EPS = 6
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
        self.msg_deque.appendleft(msg)
        # print(f"I heard about pose: {msg.header.stamp}, {msg.point}")
        # print(
        #     f"State after msg - {msg.header.stamp} is {bool(self.update())}, coordinates - {msg.point.x, msg.point.y, msg.point.z}"
        # )
        # self.update_state()
        self.state = self.update()

    def update_state(self):
        self.state = self.update()

    def update(self) -> State:
        def check_msg(input_msg) -> bool:
            # check if train is on the circle
            cur_time = datetime.datetime.timestamp(datetime.datetime.now())
            msg_timestamp = input_msg.header.stamp.sec + (input_msg.header.stamp.nanosec * 1e-9)
            return cur_time - msg_timestamp < EPS

        res: bool = True
        if len(self.msg_deque) < 5:
            return State.UNDEF
        for new_msg in self.msg_deque:
            res *= check_msg(new_msg)
        return State.VALID if res else State.INVALID

