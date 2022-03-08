import sys

import rclpy

from .dynamic_pid_reconfigurer import DynamicPIDReconfigurer


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("march_gain_scheduling_node", automatically_declare_parameters_from_overrides=True)
    node.get_logger().info("Created node")

    # rate = node.create_rate(0.5, node.get_clock())
    # try:
    #     while rclpy.ok() and not node.has_parameter("/march/joint_names"):
    #         rate.sleep()
    #         node.get_logger().debug("Waiting on /march/joint_names to be available")
    # except KeyboardInterrupt:
    #     pass
    #
    # # I think this is redundant due to the way spin works right?
    # if not rclpy.ok():
    #     return

    # joint_list = node.declare_parameter("/march/joint_names").value


    joint_list = ['left_ankle', 'left_hip_aa', 'left_hip_fe', 'left_knee', 'right_ankle',
                  'right_hip_aa', 'right_hip_fe', 'right_knee']
    DynamicPIDReconfigurer(joint_list, node)

    rclpy.spin(node)


if __name__ == "__main__":
    main()
