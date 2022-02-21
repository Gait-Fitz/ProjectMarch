"""Author: Marten Haitjema, MVII"""

import numpy as np

from rclpy.node import Node
from march_gait_selection.dynamic_interpolation.dynamic_subgait import DynamicSubgait
from march_utility.utilities.utility_functions import get_position_from_yaml
from march_utility.exceptions.gait_exceptions import (
    PositionSoftLimitError,
    VelocitySoftLimitError,
)
from geometry_msgs.msg import Point
from typing import List

ROUNDING_PRECISION = 3


class SetGaitParameterLimits:
    """Class that checks what the limits are for each
    dynamic gait parameter.

    :param gait_selection_node: the gait_selection node
    :type gait_selection_node: Node
    """

    def __init__(self, gait_selection_node: Node):
        self.gait_selection = gait_selection_node
        self.gait = gait_selection_node.dynamic_setpoint_gait
        # Fix: start position will now always be homestand
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
        self.gait.logger.info(
            f"middle_point_height: {self._limits['middle_point_height']} "
            f"middle_point_fraction: {self._limits['middle_point_fraction']} "
            f"push_off_position: {self._limits['push_off_position']} "
            f"push_off_fraction: {self._limits['push_off_fraction']} "
            f"dynamic_subgait_duration: {self._limits['dynamic_subgait_duration']}."
        )

    @property
    def limits(self) -> dict:
        return self._limits

    def loop_over_parameters(self) -> None:
        """Loops over each parameter for the range of values.
        Sets the min and max limits and writes this to a yaml file."""
        self.reset()
        self.middle_point_height = []
        middle_point_height_values = self.create_linspace(0.1, 0.5, 17)
        self.set_middle_point_height(middle_point_height_values)

        self.reset()
        self.middle_point_fraction = []
        middle_point_fraction_values = self.create_linspace(0.4, 0.7, 11)
        self.set_middle_point_fraction(middle_point_fraction_values)

        self.reset()
        self.push_off_position = []
        push_off_position_values = self.create_linspace(0.0, 0.5, 21)
        self.set_push_off_position(push_off_position_values)

        self.reset()
        self.push_off_fraction = []
        push_off_fraction_values = self.create_linspace(0.05, 0.3, 11)
        self.set_push_off_fraction(push_off_fraction_values)

        self.reset()
        self.duration = []
        times = self.create_linspace(0.5, 5.0, 91)
        self.set_duration(times)

        self._limits = {
            "middle_point_height": [
                min(self.middle_point_height),
                max(self.middle_point_height),
            ],
            "middle_point_fraction": [
                min(self.middle_point_fraction),
                max(self.middle_point_fraction),
            ],
            "push_off_position": [
                -max(self.push_off_position),
                -min(self.push_off_position),
            ],
            "push_off_fraction": [
                min(self.push_off_fraction),
                max(self.push_off_fraction),
            ],
            "dynamic_subgait_duration": [min(self.duration), max(self.duration)],
        }

    def set_middle_point_height(self, values: List[float]) -> None:
        """Set the parameter to given value and try to get trajectory.
        Append to list of possible values if successful."""
        for value in values:
            self.dynamic_subgait.middle_point_height = value
            if self.try_to_get_stair_trajectory():
                self.gait.logger.info(f"Appending value {value}")
                self.middle_point_height.append(value)

        for value in self.middle_point_height:
            if not self.try_to_get_home_stand_trajectory():
                self.gait.logger.info(f"Removing value {value}")
                self.middle_point_height.remove(value)

        for value in self.middle_point_height:
            if not self.try_to_get_normal_trajectory():
                self.gait.logger.info(f"Removing value {value}")
                self.middle_point_height.remove(value)

    def set_middle_point_fraction(self, values: List[float]) -> None:
        """Set the parameter to given value and try to get trajectory.
        Append to list of possible values if successful."""
        for value in values:
            self.dynamic_subgait.middle_point_fraction = value
            if self.try_to_get_stair_trajectory():
                self.middle_point_fraction.append(value)

        for value in self.middle_point_fraction:
            if not self.try_to_get_home_stand_trajectory():
                self.middle_point_fraction.remove(value)

        for value in self.middle_point_fraction:
            if not self.try_to_get_normal_trajectory():
                self.middle_point_fraction.remove(value)

    def set_push_off_position(self, values: List[float]) -> None:
        """Set the parameter to given value and try to get trajectory.
        Append to list of possible values if successful."""
        for value in values:
            self.dynamic_subgait.push_off_position = value
            if self.try_to_get_stair_trajectory():
                self.push_off_position.append(value)

        for value in self.push_off_position:
            if not self.try_to_get_home_stand_trajectory():
                self.push_off_position.remove(value)

        for value in self.push_off_position:
            if not self.try_to_get_normal_trajectory():
                self.push_off_position.remove(value)

    def set_push_off_fraction(self, values: List[float]) -> None:
        """Set the parameter to given value and try to get trajectory.
        Append to list of possible values if successful."""
        for value in values:
            self.dynamic_subgait.push_off_fraction = value
            if self.try_to_get_stair_trajectory():
                self.push_off_fraction.append(value)

        for value in self.push_off_fraction:
            if not self.try_to_get_home_stand_trajectory():
                self.push_off_fraction.remove(value)

        for value in self.push_off_fraction:
            if not self.try_to_get_normal_trajectory():
                self.push_off_fraction.remove(value)

    def set_duration(self, values: List[float]) -> None:
        """Set the parameter to given value and try to get trajectory.
        Append to list of possible values if successful."""
        for value in values:
            self.dynamic_subgait.dynamic_subgait_duration = value
            if self.try_to_get_stair_trajectory():
                self.duration.append(value)

        for value in self.duration:
            if not self.try_to_get_home_stand_trajectory():
                self.duration.remove(value)

        for value in self.duration:
            if not self.try_to_get_normal_trajectory():
                self.duration.remove(value)

    def try_to_get_trajectory(self) -> bool:
        """Tries to create a trajectory with the set parameters. If
        successful, it returns True, else False."""
        try:
            self.dynamic_subgait.get_joint_trajectory_msg()
        except (
            PositionSoftLimitError,
            VelocitySoftLimitError,
        ):
            return False
        return True

    def reset(self) -> None:
        """Reset parameters to values of the gait_selection_node"""
        self.dynamic_subgait.middle_point_height = (
            self.gait_selection.middle_point_height
        )
        self.dynamic_subgait.middle_point_fraction = (
            self.gait_selection.middle_point_fraction
        )
        self.dynamic_subgait.push_off_position = self.gait_selection.push_off_position
        self.dynamic_subgait.push_off_fraction = self.gait_selection.push_off_fraction

    def create_linspace(self, lower, upper, points) -> List[float]:
        """Returns a list of floats from the lower to the upper
        bound with points amount of points imbetween

        :param lower: lower bound of linspace
        :type lower: float
        :param upper: upper bound of linspace
        :type upper: float
        :param points: amount of points in linspace
        :type points: float
        """
        return [
            round(value, ROUNDING_PRECISION)
            for value in np.linspace(lower, upper, points).tolist()
        ]

    def try_to_get_stair_trajectory(self) -> bool:
        self.dynamic_subgait.location.x = 0.5
        self.dynamic_subgait.location.y = 0.15
        if self.try_to_get_trajectory():
            self.dynamic_subgait.starting_position = (
                self.dynamic_subgait.get_final_position()
            )
            return self.try_to_get_trajectory()
        else:
            return False

    def try_to_get_home_stand_trajectory(self) -> bool:
        self.dynamic_subgait.location.x = 0.4
        self.dynamic_subgait.location.y = 0.0
        self.set_start_to_home_stand()
        return self.try_to_get_trajectory()

    def try_to_get_normal_trajectory(self) -> bool:
        self.set_start_to_end()
        return self.try_to_get_trajectory()

    def set_start_to_home_stand(self) -> None:
        self.dynamic_subgait.starting_position_position = (
            self.gait.joint_dict_to_setpoint_dict(get_position_from_yaml("stand"))
        )

    def set_start_to_end(self) -> None:
        self.dynamic_subgait.starting_position = (
            self.dynamic_subgait.get_final_position()
        )
