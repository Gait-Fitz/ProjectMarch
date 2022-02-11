import yaml
import numpy as np

from time import sleep
from march_gait_selection.dynamic_interpolation.dynamic_subgait import DynamicSubgait
from march_utility.exceptions.gait_exceptions import (
    PositionSoftLimitError,
    VelocitySoftLimitError,
)
from geometry_msgs.msg import Point


class SetGaitParameterLimits:
    def __init__(self, gait_selection_node):
        sleep(2)
        self.gait = gait_selection_node.dynamic_setpoint_gait
        self.start_position = self.gait.start_position
        self.joint_soft_limits = self.gait.joint_soft_limits
        self.joint_names = self.gait.joint_names

        self.subgait_id = "right_swing"
        self.location = Point()
        self.location.x = 0.4
        self.location.y = 0.0
        stop = False

        self.dynamic_subgait = DynamicSubgait(
            gait_selection_node,
            self.start_position,
            self.subgait_id,
            self.joint_names,
            self.location,
            self.joint_soft_limits,
            stop,
        )

        self.loop_over_parameters()
        self.gait.logger.info(f"{self.default_limits}")

    def loop_over_parameters(self):
        self.reset()
        self.middle_point_height = []
        middle_point_height_values = [
            round(value, 3) for value in np.linspace(0.1, 0.5, 9).tolist()
        ]
        for value in middle_point_height_values:
            self.set_middle_point_height(value)

        self.reset()
        self.middle_point_fraction = []
        middle_point_fraction_values = [
            round(value, 3) for value in np.linspace(0.4, 0.7, 6).tolist()
        ]
        for value in middle_point_fraction_values:
            self.set_middle_point_fraction(value)

        self.reset()
        self.push_off_position = []
        push_off_position_values = [
            round(value, 3) for value in np.linspace(0.0, 0.5, 11).tolist()
        ]
        for value in push_off_position_values:
            self.set_push_off_position(value)

        self.reset()
        self.push_off_fraction = []
        push_off_fraction_values = [
            round(value, 3) for value in np.linspace(0.05, 0.3, 6).tolist()
        ]
        for value in push_off_fraction_values:
            self.set_push_off_fraction(value)

        self.reset()
        self.duration = []
        times = [round(value, 3) for value in np.linspace(0.5, 5.0, 91).tolist()]
        for time in times:
            self.set_duration(time)

        self.default_limits = {
            "middle_point_height": [
                min(self.middle_point_height),
                max(self.middle_point_height),
            ],
            "middle_point_fraction": [
                min(self.middle_point_fraction),
                max(self.middle_point_fraction),
            ],
            "limit_push_off_position": [
                min(self.push_off_position),
                max(self.push_off_position),
            ],
            "limit_push_off_fraction": [
                min(self.push_off_fraction),
                max(self.push_off_fraction),
            ],
            "limit_duration": [min(self.duration), max(self.duration)],
        }

        path = "src/gaits/march_gait_selection/march_gait_selection/dynamic_interpolation/dynamic_gait_limits.yaml"
        with open(path, "w") as yaml_file:
            yaml_file.write(yaml.dump(self.default_limits))

    def set_middle_point_height(self, value):
        self.dynamic_subgait.middle_point_height = value
        if self.try_to_get_trajectory():
            self.middle_point_height.append(value)

    def set_middle_point_fraction(self, value):
        self.dynamic_subgait.middle_point_fraction = value
        if self.try_to_get_trajectory():
            self.middle_point_fraction.append(value)

    def set_push_off_position(self, value):
        self.dynamic_subgait.push_off_position = -value
        if self.try_to_get_trajectory():
            self.push_off_position.append(value)

    def set_push_off_fraction(self, value):
        self.dynamic_subgait.push_off_fraction = value
        if self.try_to_get_trajectory():
            self.push_off_fraction.append(value)

    def set_duration(self, value):
        self.dynamic_subgait.duration = value
        if self.try_to_get_trajectory():
            self.duration.append(value)

    def try_to_get_trajectory(self):
        try:
            self.dynamic_subgait.get_joint_trajectory_msg()
        except (
            PositionSoftLimitError,
            VelocitySoftLimitError,
        ):
            return False
        return True

    def reset(self):
        self.dynamic_subgait.middle_point_height = 0.15
        self.dynamic_subgait.middle_point_fraction = 0.45
        self.dynamic_subgait.push_off_position = -0.15
        self.dynamic_subgait.push_off_fraction = 0.2
