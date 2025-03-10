import time
from typing import List, Dict
import datetime


class SimpleModel:
    pass


def model_state_down(model_name: str, layer: "Layer", sleep_time: float) -> None:
    if layer:
        for model in layer.models:
            time.sleep(sleep_time)
            if model_name == model.name:
                model.state = False
                model_state_down(model_name, model.layer, sleep_time)


class Layer:
    def __init__(
        self,
        layer_name: str,
        layer_number: int,
        level_NM: int,
        idle_models: List[str],
        sleep_time: float,
    ):
        self.layer_name: str = layer_name
        self.layer_number: int = layer_number
        self.sleep_time: float = sleep_time
        self.models: List[SimpleModel] = (
            [
                SimpleModel(
                    name=layer_name + f"_{index}",
                    layer_index=layer_number - 1,
                    level_NM=level_NM,
                    idle_models=idle_models,
                    sleep_time=sleep_time,
                )
                for index in range(level_NM)
            ]
            if layer_number >= 1
            else None
        )
        # time.sleep(sleep_time)

    def update_children(self, request_vector: List[str]):
        for model in self.models:
            if model.name in request_vector:
                print(f"Updating model - {model.name}")
                model.state = True
                if model.layer is not None:
                    model.layer.update_children(request_vector)

    def get_children_state(self) -> bool:
        res: bool = True
        for model in self.models:
            res *= model.state
            if model.layer:
                res *= model.layer.get_children_state()
        return bool(res)

    def instance_update(self, request_vector: List[str], sleep_time: float) -> bool:
        res: bool = True
        for model in self.models:
            if model.name in request_vector:
                if model.layer:
                    print("Trying to update")
                    model.instant_update(request_vector, sleep_time=sleep_time)
                    res *= model.layer.get_children_state()
                else:
                    res *= False
        return res

    def __bool__(self):
        return True if self.layer_number else False


class SimpleModel:

    def __init__(
        self,
        name: str,
        layer_index: int,
        level_NM: int,
        idle_models: List[str],
        sleep_time: float,
    ):
        self.name: str = name
        self.state: bool = True
        self.level_NM: int = level_NM
        self.idle_models: List[str] = idle_models
        self.idle: bool = True if any([name == el for el in idle_models]) else False
        self.layer_index: int = layer_index
        self.sleep_time: float = sleep_time
        self.layer: Layer = (
            Layer(
                layer_name=f"{self.name}",
                layer_number=layer_index,
                level_NM=level_NM,
                idle_models=idle_models,
                sleep_time=self.sleep_time,
            )
            if layer_index >= 1
            else None
        )
        time.sleep(sleep_time)

    def update(self) -> bool:
        res: bool = bool(self.state * self.layer.get_children_state())
        return res

    def idle_update(self, _, sleep_time: float):
        if self.layer:
            for model in self.layer.models:
                if model.idle:
                    time.sleep(sleep_time)
                    model.state = True
                model.idle_update(_, sleep_time=sleep_time)

    def check_model(self) -> bool:
        if self.idle:
            self.state = self.update()
        return self.state

    def instant_update(self, request_vector: List[str], sleep_time: float):
        if self.layer:
        # if self.layer.get_children_state():
            for model in self.layer.models:
                time.sleep(sleep_time)
                # print(model.name)
                if model.name in request_vector:
                    self.state = bool(
                        self.state
                        * self.layer.instance_update(request_vector, sleep_time)
                    )
        # self.update()

    def delayed_update(self, request_vector: List[str], sleep_time: float):
        for model_name in request_vector:
            if self.layer:
                for model in self.layer.models:
                    time.sleep(sleep_time)
                    if model.name == model_name:
                        model_state_down(model_name, model.layer, sleep_time)
        self.layer: Layer = (
            Layer(
                layer_name=f"{self.name}",
                layer_number=self.layer_index,
                level_NM=self.level_NM,
                idle_models=self.idle_models,
                sleep_time=self.sleep_time,
            )
            if self.layer_index >= 1
            else None
        )


def get_hierarchy(model: SimpleModel) -> str:
    res: str = f"{model.name, model.idle} "
    if model.layer:
        res: str = f"{model.name}:\n"
        # res += " , ".join([f"{el.name}" for el in model.layer.models]) + "\n"
        if model.layer.models:
            for child_model in model.layer.models:
                res += (
                    f"layer - {model.layer.layer_name}, model - "
                    + get_hierarchy(child_model)
                    + "\n"
                )
    return res
