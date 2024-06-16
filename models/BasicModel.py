from typing import List, Dict
from defs import State

class BasicModel(object):
    def __init__(self):
        self.state: State = State.UNDEF
        self.children_models: Dict[str, BasicModel] = dict()

    def update_state(self):
        if self.state == State.INVALID:
            for child in self.children_models.values():
                if child.state == State.INVALID or child.state == State.UNDEF:
                    child.update_state()

    def check_children(self) -> State:
        res :State = State.UNDEF
        for child in self.children_models.values():
            if child.state == State.VALID:
                res = State.VALID
            elif child.state == State.INVALID:
                res = State.INVALID
            else:
                child.update_state()
        return res