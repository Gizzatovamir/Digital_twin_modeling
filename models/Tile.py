from models.BasicModel import BasicModel
from utils.Config import Config
from models.Train import Train
import rclpy


class Tile(BasicModel):
    def __init__(self, config: Config, **kwargs):
        self.config: Config = config
        super().__init__(**kwargs)
        self.synthesize(
            self.config.n_children, self.config.name, self.config.topic_name, Train
        )

    def delayed_update(self):
        self.synthesize(
            self.config.n_children, self.config.name, self.config.topic_name, Train
        )


