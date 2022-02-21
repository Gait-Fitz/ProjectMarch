"""Author: Marten Haitjema, MVII"""

import numpy as np

from rclpy.node import Node
from march_gait_selection.dynamic_interpolation.dynamic_joint_trajectory import (
    DynamicJointTrajectory,
)
from march_utility.gait.limits import Limits
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.duration import Duration
from march_utility.utilities.utility_functions import get_position_from_yaml
from march_utility.utilities.logger import Logger
from march_goniometric_ik_solver.ik_solver import Pose

from trajectory_msgs import msg as trajectory_msg
from geometry_msgs.msg import Point

from typing import List
from enum import IntEnum

EXTRA_ANKLE_SETPOINT_INDEX = 1
INTERPOLATION_POINTS = 30


class SetpointTime(IntEnum):
    START_INDEX = 0
    PUSH_OFF_INDEX = 1
    MIDDLE_POINT_INDEX = 2
    END_POINT_INDEX = 3


class DynamicSubgait:
    """Creates joint trajectories based on the desired foot location.

    :param gait_selection_node: The gait_selection node
    :type gait_selection: Node
    :param starting_position: The first setpoint of the subgait, usually the last setpoint of the previous subgait.
    :type starting_position: dict
    :param subgait_id: Whether it is a left_swing or right_swing.
    :type subgait_id: str
    :param joint_names: Names of the joints
    :type joint_names: list
    :param position: Desired foot position
    :type position: Point
    :param joint_soft_limits: list containing soft limits in alphabetical order
    :type joint_soft_limits: List[Limits]
    :param stop: whether it is a close gait or not
    :type stop: bool
    """

    def __init__(
        self,
        gait_selection_node: Node,
        starting_position: dict,
        subgait_id: str,
        joint_names: List[str],
        location: Point,
        joint_soft_limits: List[Limits],
        stop: bool,
    ):
        self.logger = Logger(gait_selection_node, __class__.__name__)
        self._get_parameters(gait_selection_node)
        self.time = [
            0,
            self.push_off_fraction * self.duration,
            self.middle_point_fraction * self.duration,
            self.duration,
        ]
        self.starting_position = starting_position
        self.location = location
        self.joint_names = joint_names
        self.subgait_id = subgait_id
        self.joint_soft_limits = joint_soft_limits
        self.stop = stop

    def _get_extra_ankle_setpoint(self) -> Setpoint:
        """Returns an extra setpoint for the swing leg ankle
        that can be used to create a push off.

        :returns: An extra setpoint for the swing leg ankle
        :rtype: Setpoint
        """
        return Setpoint(
            Duration(self.time[SetpointTime.PUSH_OFF_INDEX]),
            self.push_off_position,
            0.0,
        )

    def _solve_middle_setpoint(self) -> None:
        """Calls IK solver to compute the joint angles needed for the middle setpoint

        :returns: A setpoint_dict for the middle position.
        :rtype: dict
        """
        middle_position = self.pose.solve_mid_position(
            self.location.x,
            self.location.y,
            self.middle_point_fraction,
            self.middle_point_height,
            self.subgait_id,
        )

        self.middle_setpoint_dict = self._from_list_to_setpoint(
            self.joint_names,
            middle_position,
            None,
            self.time[SetpointTime.MIDDLE_POINT_INDEX],
        )

    def _solve_desired_setpoint(self) -> None:
        """Calls IK solver to compute the joint angles needed for the
        desired x and y coordinate"""
        if self.stop:
            self.desired_position = self._from_joint_dict_to_list(
                get_position_from_yaml("stand")
            )
        else:
            self.desired_position = self.pose.solve_end_position(
                self.location.x, self.location.y, self.subgait_id
            )
        desired_velocity = np.zeros_like(self.desired_position)
        # if self.subgait_id == "right_swing":
        #     desired_velocity[6] = -0.28
        # else:
        #     desired_velocity[2] = -0.28
        self.desired_setpoint_dict = self._from_list_to_setpoint(
            self.joint_names,
            self.desired_position,
            desired_velocity,
            self.time[SetpointTime.END_POINT_INDEX],
        )

    def _to_joint_trajectory_class(self) -> None:
        """Creates a list of DynamicJointTrajectories for each joint"""
        self.joint_trajectory_list = []
        for name in self.joint_names:
            setpoint_list = [
                self.starting_position[name],
                self.middle_setpoint_dict[name],
                self.desired_setpoint_dict[name],
            ]

            # Add an extra setpoint to the ankle to create a push off:
            if (name == "right_ankle" and self.subgait_id == "right_swing") or (
                name == "left_ankle" and self.subgait_id == "left_swing"
            ):
                setpoint_list.insert(
                    EXTRA_ANKLE_SETPOINT_INDEX, self._get_extra_ankle_setpoint()
                )
            
            if name in ["right_ankle", "left_ankle"]:
                self.joint_trajectory_list.append(
                    DynamicJointTrajectory(setpoint_list, ankle=True)
                )
            else:
                self.joint_trajectory_list.append(DynamicJointTrajectory(setpoint_list))

    def get_joint_trajectory_msg(self) -> trajectory_msg.JointTrajectory:
        """Return a joint_trajectory_msg containing the interpolated
        trajectories for each joint

        :returns: A joint_trajectory_msg
        :rtype: joint_trajectory_msg
        """
        # Create a pose object of last gait:
        self.pose = Pose([joint.position for joint in self.starting_position.values()])

        # In case of a right_swing, we need to swap sides of the pose object from last gait:
        if self.subgait_id == "right_swing":
            self.pose.swap_sides()

        # Calculate the mid and end position for current gait:
        self._solve_middle_setpoint()
        self._solve_desired_setpoint()
        self._get_extra_ankle_setpoint()

        # Create joint_trajectory_msg
        self._to_joint_trajectory_class()
        joint_trajectory_msg = trajectory_msg.JointTrajectory()
        joint_trajectory_msg.joint_names = self.joint_names

        timestamps = np.linspace(self.time[0], self.time[-1], INTERPOLATION_POINTS)
        for timestamp in timestamps:
            joint_trajecory_point = trajectory_msg.JointTrajectoryPoint()
            joint_trajecory_point.time_from_start = Duration(timestamp).to_msg()

            for joint_index, joint_trajectory in enumerate(self.joint_trajectory_list):
                interpolated_setpoint = joint_trajectory.get_interpolated_setpoint(
                    timestamp
                )

                joint_trajecory_point.positions.append(interpolated_setpoint.position)
                joint_trajecory_point.velocities.append(interpolated_setpoint.velocity)
                self._check_joint_limits(joint_index, joint_trajecory_point)

            joint_trajectory_msg.points.append(joint_trajecory_point)

        return joint_trajectory_msg

    def get_final_position(self) -> dict:
        """Get setpoint_dictionary of the final setpoint.

        :return: The final setpoint of the subgait.
        :rtype: dict
        """
        return self._from_list_to_setpoint(
            self.joint_names,
            self.desired_position,
            None,
            self.time[SetpointTime.START_INDEX],
        )

    def _from_list_to_setpoint(
        self,
        joint_names: List[str],
        position: List[float],
        velocity: List[float],
        time: float,
    ) -> dict:
        """Computes setpoint_dictionary from a list

        :param joint_names: Names of the joints.
        :type joint_names: list
        :param position: Positions for each joint.
        :type position: list
        :param velocity: Optional velocities for each joint. If None, velocity will be set to zero.
        :type velocity: list
        :param time: Time at which the setpoint should be set.
        :type time: float

        :returns: A Setpoint_dict containing time, position and velocity for each joint
        :rtype: dict
        """
        setpoint_dict = {}
        velocity = np.zeros_like(position) if (velocity is None) else velocity

        for i, name in enumerate(joint_names):
            if (name == "right_ankle" and self.subgait_id == "right_swing") or (
                name == "left_ankle" and self.subgait_id == "left_swing"
            ):
                velocity[i] = 0.0

            setpoint_dict.update(
                {
                    joint_names[i]: Setpoint(
                        Duration(time),
                        position[i],
                        velocity[i],
                    )
                }
            )

        return setpoint_dict

    def _from_joint_dict_to_list(self, joint_dict: dict) -> List[float]:
        """Return the values in a joint_dict as a list."""
        return list(joint_dict.values())

    def _get_parameters(self, gait_selection_node: Node) -> None:
        """Gets the dynamic gait parameters from the gait_selection_node

        :param gait_selection_node: the gait selection node
        :type gait_selection_node: Node
        """
        self.duration = gait_selection_node.dynamic_subgait_duration
        self.middle_point_height = gait_selection_node.middle_point_height
        self.middle_point_fraction = gait_selection_node.middle_point_fraction
        self.push_off_fraction = gait_selection_node.push_off_fraction
        self.push_off_position = gait_selection_node.push_off_position

    def _check_joint_limits(
        self,
        joint_index: int,
        joint_trajectory_point: trajectory_msg.JointTrajectoryPoint,
    ) -> None:
        """Check if values in the joint_trajectory_point are within the soft and
        velocity limits defined in the urdf

        :param joint_index: Index of the joint in the alphabetical joint_names list
        :type joint_index: int
        :param joint_trajectory_point: point in time containing position and velocity
        :type joint_trajectory_point: trajectory_msg.JointTrajectoryPoint
        """
        position = joint_trajectory_point.positions[joint_index]
        velocity = joint_trajectory_point.velocities[joint_index]
        if (
            position > self.joint_soft_limits[joint_index].upper
            or position < self.joint_soft_limits[joint_index].lower
        ):
            self.logger.info(
                f"DynamicSubgait: {self.joint_names[joint_index]} will be outside of soft limits, "
                f"position: {position}, soft limits: "
                f"[{self.joint_soft_limits[joint_index].lower}, {self.joint_soft_limits[joint_index].upper}]."
            )
            raise Exception(
                f"{self.joint_names[joint_index]} will be outside its soft limits."
            )

        if abs(velocity) > self.joint_soft_limits[joint_index].velocity:
            self.logger.info(
                f"DynamicSubgait: {self.joint_names[joint_index]} will be outside of velocity limits, "
                f"velocity: {velocity}, velocity limit: {self.joint_soft_limits[joint_index].velocity}."
            )
            raise Exception(
                f"{self.joint_names[joint_index]} will be outside its velocity limits."
            )
