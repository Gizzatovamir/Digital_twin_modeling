import rclpy
from models.BasicModel import BasicModel
from utils.Config import Config
from models.Train import Train
from models.Tile import Tile
from utils.defs import state_string_dict
from rclpy.logging import LoggingSeverity
import time
from typing import List
import numpy as np
from rclpy.node import Node

PERIOD = 0.05
rclpy.logging._root_logger.set_level(LoggingSeverity.ERROR)


if __name__ == "__main__":
    # config = Config(3, "test", f"/point_stamped_", 0, ["test_1"])
    config = Config(3, "test", f"/point_stamped_", 0, [])
    # config = Config(3, "test", f"/point_stamped_", 0, [])
    rclpy.init()
    tile = Tile(config)
    i: int = 0
    train_to_update_1: str = "test_0"
    train_to_update_2: str = "test_1"
    time_list: List[float] = list()
    start_i = 10
    end_i = 30

    while rclpy.ok():
        i += 1
        if i > start_i and i < end_i:
            # tile.delayed_update([train_to_update_1])
            # print(
            #     f"Made delayed update for {train_to_update_1} state - {state_string_dict.get(tile.children_models[train_to_update_1].state)}"
            # )
            tile.children_models[train_to_update_1].immediate_update(train_to_update_1)
            tile.children_models[train_to_update_2].immediate_update(train_to_update_2)
            print(
                f"Made instant update for {train_to_update_2}, state - {state_string_dict.get(tile.children_models[train_to_update_2].state)}"
            )
            print(
                f"Made instant update for {train_to_update_1}, state - {state_string_dict.get(tile.children_models[train_to_update_1].state)}"
            )

        # if i == 10:
        #     print(f'Made instant update for {train_to_update}')
        #     tile.immediate_update(train_to_update)
        now = Node("now").get_clock().now().nanoseconds
        for train in tile.children_models.values():
            rclpy.spin_once(train, timeout_sec=0.5)
            # print(f"[{(train.get_clock().now().nanoseconds - now) * 1e-9}] ", end=' ')
            print(
                f"Vehicle - {train.name} is {state_string_dict.get(train.state)}",
                end=" ",
            )
            if train.idle:
                print(f"\n [{(train.get_clock().now().nanoseconds - now) * 1e-6}] ", end=' ')
                print(
                    f"Vehicle - {train.name} is in idle mode and updates frequantly, current state - {state_string_dict.get(train.state)}"
                )
                # start_time = time.time()
                # train.update_state()
                # exec_time = time.time() - start_time
                # print("--- %s seconds ---" % exec_time)
                # time_list.append(exec_time)
        print(f"\n Tile - {tile.name} is {state_string_dict.get(tile.state)}")
        # print("\n")
        tile.update_state()

        # except Exception as e:
        #     print(f"something went wrong in the ROS Loop: {e}")

    rclpy.shutdown()
