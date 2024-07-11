from typing import Dict, List, Union, TYPE_CHECKING, Callable
from utils.defs import State

if TYPE_CHECKING:
    from models import Tile, Map, Train


class BasicModel(object):
    def __init__(self, **kwargs):
        self.state: State = State.UNDEF
        self.children_models: Dict[str, BasicModel] = dict()

    def update_state(self):
        if self.state == State.INVALID:
            for child in self.children_models.values():
                if child.state == State.INVALID or child.state == State.UNDEF:
                    child.update_state()

    def check_children(self) -> State:
        res: State = State.UNDEF
        for child in self.children_models.values():
            if child.state == State.VALID:
                res = State.VALID
            elif child.state == State.INVALID:
                res = State.INVALID
            else:
                child.update_state()
        return res

    def synthesize(
        self,
        n: int,
        name: str,
        topic_name: str,
        child_class: Callable,
    ):
        for i in range(n):
            child_topic_name = f"{topic_name}_{name}_{i}"
            child_name = f"{name}_{i}"
            print(child_topic_name)
            print(child_name)
            self.children_models[child_name] = child_class(child_name, child_topic_name)
