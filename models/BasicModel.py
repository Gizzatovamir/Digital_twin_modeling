from typing import Dict, List, Union, TYPE_CHECKING, Callable
from utils.defs import State

if TYPE_CHECKING:
    from models import Tile, Map
    from models.Train import Train


class BasicModel(object):
    def __init__(self, **kwargs):
        self.state: State = State.UNDEF
        self.children_models: Dict[str, BasicModel] = dict()
        self.name: str = ""
        self.idle: bool = False
        self.topic_name: str = ""

    def immediate_update(self, child_to_update: str) -> State:
        res: State = State.UNDEF
        if self.children_models:
            for child_name, child_instance in self.children_models.items():
                if child_name == child_to_update:
                    child_instance.update_state()
                    res = child_instance.state
                else:
                    res = child_instance.immediate_update(child_to_update)
        else:
            if self.name == child_to_update:
                self.update_state()
        return res

    def update_state(self):
        state_buff: List[bool] = list()
        for child in self.children_models.values():
            if child.state == State.INVALID or child.state == State.UNDEF:
                state_buff.append(child.update())
            # else:
            #     state_buff.append(True)
        self.state = State.VALID if all(state_buff) else State.INVALID

    def check_children(self) -> bool:
        res: bool = False
        # [el.update() for el in self.children_models.values()]
        for child in self.children_models.values():
            if child.state == State.VALID:
                res = True
            elif child.state == State.INVALID:
                res = False
            else:
                child.update_state()
        return res

    def synthesize(
        self, n: int, name: str, topic_name: str, child_class: Callable, **kwargs
    ):
        to_synth: List[str] = kwargs.get("to_synth", [])
        if len(to_synth) == 0:
            for i in range(n):
                child_topic_name = f"{topic_name}{name}_{i}"
                child_name = f"{name}_{i}"
                if kwargs["call_class"] == "train":
                    model_name = kwargs.get("idle")
                    if child_name in model_name:
                        idle = True
                    else:
                        idle = False
                    self.children_models[child_name] = child_class(
                        child_name, child_topic_name, idle=idle
                    )
                else:
                    self.children_models[child_name] = child_class(kwargs["config"])
        else:
            for child in to_synth:
                child_instance = self.children_models[child]
                self.children_models[child] = child_class(
                    child_instance.name,
                    child_instance.topic_name,
                    idle=child_instance.idle,
                )
