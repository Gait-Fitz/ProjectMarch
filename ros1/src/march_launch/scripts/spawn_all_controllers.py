#!/usr/bin/env python3
from typing import List

import rospy
from roslaunch.core import Node
from roslaunch.scriptapi import ROSLaunch

NAMESPACE = "march"
CONTROLLER_NAMESPACE = "controller"


def main():
    """This script looks into the ros parameter server and spawns all controller it can find."""
    try:
        rospy.init_node("spawn_all_controllers")
    except rospy.ROSInitException:
        return

    all_controllers_config: dict = rospy.get_param(
        f"/{NAMESPACE}/{CONTROLLER_NAMESPACE}"
    )

    num_trajectory_controllers = 0
    controllers_list: List[List[str]] = [[]]

    for controller, config in all_controllers_config.items():
        controller_spawn_name = f" {CONTROLLER_NAMESPACE}/{controller}"
        # if "JointTrajectoryController" in config["type"]:
        #     if num_trajectory_controllers == len(controllers_list):
        #         controllers_list.append([])
        #     controllers_list[num_trajectory_controllers].append(controller_spawn_name)
        #     num_trajectory_controllers += 1
        # else:
        controllers_list[0].append(controller_spawn_name)

    nodes: List[Node] = []
    for i, controllers in enumerate(controllers_list):
        nodes.append(
            Node(
                package="controller_manager",
                node_type="controller_manager",
                namespace=NAMESPACE,
                name=f"controller_spawner_{i}",
                args=f"spawn {' '.join(controllers)}",
            )
        )
    launch = ROSLaunch()
    launch.start()

    for node in nodes:
        launch.launch(node)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
