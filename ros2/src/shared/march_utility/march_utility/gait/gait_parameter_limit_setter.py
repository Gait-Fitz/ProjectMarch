import numpy as np
from time import sleep
from march_gait_selection.dynamic_interpolation.dynamic_subgait import DynamicSubgait
from march_utility.exceptions.gait_exceptions import (
    PositionSoftLimitError,
    VelocitySoftLimitError,
    AccelerationSoftLimitError,
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
        values = [round(value, 3) for value in np.linspace(0.05, 1.0, 20).tolist()]
        self.middle_point_height = []
        self.middle_point_fraction = []
        self.push_off_position = []
        self.push_off_fraction = []

        for value in values:
            if value <= 0.3:
                self.set_push_off_fraction(value)
            if value <= 0.5:
                self.set_push_off_position(value)
            if value >= 0.1 and value <= 0.5:
                self.set_middle_point_height(value)
            if value >= 0.4 and value <= 0.7:
                self.set_middle_point_fraction(value)

        times = [round(value, 3) for value in np.linspace(0.5, 5.0, 50).tolist()]
        self.duration = []
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

    def set_middle_point_height(self, value):
        self.dynamic_subgait.middle_point_height = value
        if self.try_to_get_trajectory():
            self.middle_point_height.append(value)
        self.reset()

    def set_middle_point_fraction(self, value):
        self.dynamic_subgait.middle_point_fraction = value
        if self.try_to_get_trajectory():
            self.middle_point_fraction.append(value)
        self.reset()

    def set_push_off_position(self, value):
        self.dynamic_subgait.push_off_position = -value
        if self.try_to_get_trajectory():
            self.push_off_position.append(value)
        self.reset()

    def set_push_off_fraction(self, value):
        self.dynamic_subgait.push_off_fraction = value
        if self.try_to_get_trajectory():
            self.push_off_fraction.append(value)
        self.reset()

    def set_duration(self, value):
        self.dynamic_subgait.duration = value
        if self.try_to_get_trajectory():
            self.duration.append(value)
        self.reset()

    def try_to_get_trajectory(self):
        try:
            self.dynamic_subgait.get_joint_trajectory_msg()
        except (
            PositionSoftLimitError,
            VelocitySoftLimitError,
            AccelerationSoftLimitError,
        ):
            return False
        return True

    def reset(self):
        self.dynamic_subgait.middle_point_height = 0.15
        self.dynamic_subgait.middle_point_fraction = 0.45
        self.dynamic_subgait.push_off_position = -0.15
        self.dynamic_subgait.push_off_fraction = 0.2
