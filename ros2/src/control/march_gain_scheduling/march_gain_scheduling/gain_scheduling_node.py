import sys

import rclpy

from march_utility.utilities.utility_functions import get_joint_names_from_urdf
from .dynamic_pid_reconfigurer import DynamicPIDReconfigurer


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("march_gain_scheduling_node", automatically_declare_parameters_from_overrides=True)
    node.get_logger().info("Created node")
    DynamicPIDReconfigurer(get_joint_names_from_urdf(), node)

    rclpy.spin(node)


if __name__ == "__main__":
    main()
