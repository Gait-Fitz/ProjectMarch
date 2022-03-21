"""Author: Control, MIV; George Vegelien, MVII."""
import sys

import rclpy

from march_utility.utilities.utility_functions import get_joint_names_from_urdf
from .dynamic_pid_reconfigurer import DynamicPIDReconfigurer


def main():
    """Starts the "march_gain_scheduling_node" and node and passes it to DynamicPIDReconfigurer."""
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("march_gain_scheduling_node", automatically_declare_parameters_from_overrides=True)
    node.get_logger().info("Created node")
    DynamicPIDReconfigurer(get_joint_names_from_urdf(), node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
