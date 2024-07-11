from utils.defs import ChildType


class Config:
    __slots__ = ("n_children", "name", "topic_name")

    def __init__(self, n_children: int, name: str, topic_name: str):
        self.n_children: int = n_children
        self.name: str = name
        self.topic_name: str = topic_name
