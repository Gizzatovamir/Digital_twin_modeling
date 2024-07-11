from publisher import DataPublisher
import argparse
import rclpy

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", help="model name")
    args = parser.parse_args()
    rclpy.init()
    publisher = DataPublisher(0.1, args.name)
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()
