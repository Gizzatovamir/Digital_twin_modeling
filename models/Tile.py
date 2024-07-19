from models.BasicModel import BasicModel
from utils.Config import Config
from models.Train import Train
from typing import List
import rclpy


class Tile(BasicModel):
    def __init__(self, config: Config, **kwargs):
        super().__init__(**kwargs)
        self.config: Config = config
        self.name: str = "Tile_" + config.name
        self.synthesize(
            self.config.n_children, self.config.name, self.config.topic_name, Train, call_class="train", idle=config.models_to_monitor
        )


    def delayed_update(self, target_models: List[str]):
        for model_name in target_models:
            self.synthesize(
                self.config.n_children, self.config.name, self.config.topic_name, Train, call_class="train", idle=[], child_name=model_name
            )


