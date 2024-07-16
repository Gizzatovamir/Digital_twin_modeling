from models.BasicModel import BasicModel
from utils.Config import Config
from models.Tile import Tile


class Map(BasicModel):
    def __init__(self, config: Config, **kwargs):
        self.config: Config = config
        super().__init__()
        self.synthesize(
            self.config.n_children, self.config.name, self.config.topic_name, Tile, call_class="tile"
        )

    def delayed_update(self):
        config = Config(
            self.config.children_n_children,
            self.config.name + "Tile",
            self.config.topic_name,
            self.config.children_n_children,
        )
        self.synthesize(
            self.config.n_children,
            self.config.name,
            self.config.topic_name,
            Tile,
            config=config,
            call_class="tile"
        )
