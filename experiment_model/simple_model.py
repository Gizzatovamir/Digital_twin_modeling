from typing import List, Dict


class SimpleModel:
    pass


class Layer:
    def __init__(self, layer_name: str, layer_number: int, level_NM: int):
        self.layer_name: str = layer_name
        self.layer_number: int = layer_number
        self.models: List[SimpleModel] = (
            [
                SimpleModel(
                    name=layer_name + f"_{index}",
                    layer_index=layer_number - 1,
                    level_NM=level_NM,
                )
                for index in range(level_NM)
            ]
            if layer_number
            else None
        )

    def __bool__(self):
        return True if self.layer_number else False


class SimpleModel:
    def __init__(self, name: str, layer_index: int, level_NM: int):
        self.name: str = name
        self.state: bool = False
        self.layer: Layer = (
            Layer(
                layer_name=f"{self.name}",
                layer_number=layer_index,
                level_NM=level_NM,
            )
            if layer_index >= 1
            else None
        )


def get_hierarchy(model: SimpleModel) -> str:
    res: str = f"{model.name} "
    if model.layer:
        res: str = f"{model.name}:\n"
        # res += " , ".join([f"{el.name}" for el in model.layer.models]) + "\n"
        if model.layer.models:
            for child_model in model.layer.models:
                res += f"layer - {model.layer.layer_name}, model - " + get_hierarchy(child_model) + "\n"
    return res


if __name__ == "__main__":
    test_model = 2("test", 2, 4)
    print(get_hierarchy(test_model))
