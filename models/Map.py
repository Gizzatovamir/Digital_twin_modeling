from models.BasicModel import BasicModel
from utils.Config import Config


class Map(BasicModel):
    def __init__(self, config: Config, **kwargs):
        self.config: Config = config
        super().__init__()
