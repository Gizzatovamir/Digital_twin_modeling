from models.BasicModel import BasicModel
from utils.Config import Config
from models.Train import Train
import rclpy


class Tile(BasicModel):
    def __init__(self, config: Config, **kwargs):
        super().__init__(**kwargs)
        self.config: Config = config
        self.name: str = "Tile_" + config.name
        self.synthesize(
            self.config.n_children, self.config.name, self.config.topic_name, Train, call_class="train", idle=config.models_to_monitor
        )


    def delayed_update(self):
        self.synthesize(
            self.config.n_children, self.config.name, self.config.topic_name, Train, call_class="train", idle=[]
        )


