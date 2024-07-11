import rclpy
from models.BasicModel import BasicModel
from utils.Config import Config
from models.Train import Train
from models.Tile import Tile

PERIOD = 0.1
state_string_dict = {0: "UNDEF", 1: "VALID", 2: "INVALID"}


if __name__ == "__main__":
    config = Config(2, "test", f"/point_stamped")
    rclpy.init()
    tile = Tile(config)
    i: int = 0
    while rclpy.ok():
        # try:
        i += 1
        if i == 10:
            print(f"Made delayed update. Synthesize train models")
            tile.delayed_update()
        for train in tile.children_models.values():
            rclpy.spin_once(train, timeout_sec=1)
            print(
                f"Train - {train.name} is {state_string_dict.get(train.state)}", end=" "
            )
        print("\n")

        # except Exception as e:
        #     print(f"something went wrong in the ROS Loop: {e}")

    rclpy.shutdown()
