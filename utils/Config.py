from utils.defs import ChildType
from typing import List


class Config:
    __slots__ = ("n_children", "name", "topic_name", "children_n_children", "models_to_monitor")

    def __init__(self, n_children: int, name: str, topic_name: str, children_n_children: int, models_to_monitor: List[str]):
        self.n_children: int = n_children
        self.children_n_children : int = children_n_children
        self.name: str = name
        self.topic_name: str = topic_name
        self.models_to_monitor: str = models_to_monitor
