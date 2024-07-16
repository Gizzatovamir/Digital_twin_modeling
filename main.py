import rclpy
from models.BasicModel import BasicModel
from utils.Config import Config
from models.Train import Train
from models.Tile import Tile
from utils.defs import state_string_dict
from rclpy.logging import LoggingSeverity

PERIOD = 0.1
rclpy.logging._root_logger.set_level(LoggingSeverity.ERROR)


if __name__ == "__main__":
    config = Config(2, "test", f"/point_stamped", 0, ["test_1"])
    rclpy.init()
    tile = Tile(config)
    i: int = 0
    train_to_update: str = "test_1"
    while rclpy.ok():
        # try:
        i += 1
        if i == 10:
            print(f"Made delayed update. Synthesize train models")
            tile.delayed_update()
        # if i == 10:
        #     print(f'Made instant update for {train_to_update}')
        #     tile.immediate_update(train_to_update)
        for train in tile.children_models.values():
            rclpy.spin_once(train, timeout_sec=1)
            print(
                f"Train - {train.name} is {state_string_dict.get(train.state)}", end=" "
            )
            if train.idle:
                print(f"\n Train - {train.name} is in idle mode and updates frequantly")
                train.update_state()
        print("\n")
        tile.update_state()
        print(f"Tile - {tile.name} is {state_string_dict.get(tile.state)}")

        # except Exception as e:
        #     print(f"something went wrong in the ROS Loop: {e}")

    rclpy.shutdown()
