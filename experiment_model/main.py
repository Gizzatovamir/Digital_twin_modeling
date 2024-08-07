from model import Model
from pathlib import Path
from model_config import ModelConfig
import yaml

def get_config(path: Path) -> ModelConfig:
    with open(path.as_posix(), 'r') as file:
        cfg = yaml.safe_load(file)
        return ModelConfig(**cfg)

if __name__ == '__main__':
    config_path: Path = Path("config.yaml")
    config: ModelConfig = get_config(config_path)
    print(config)