import numpy as np

from march_gait_selection.dynamic_interpolation.dynamic_joint_trajectory import (
    DynamicJointTrajectory,
)
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.duration import Duration
from trajectory_msgs import msg as trajectory_msg
from march_goniometric_ik_solver.ik_solver import Pose


class DynamicSubgait:
    """Creates joint trajectories based on the desired foot location.

    :param duration: Duration of the subgait.
    :type duration: float
    :param mid_point_fraction: Fraction of the subgait at which the middle setpoint will be set.
    :type mid_point_fraction: float
    :param starting_position: The first setpoint of the subgait, usually the last setpoint of the previous subgait.
    :type starting_position: dict
    :param subgait_id: Whether it is a left_swing or right_swing.
    :type subgait_id: str
    :param position_x: x-coordinate of the desired foot location in meters.
    :type position_x: float
    :param position_y: Optional y-coordinate of the desired foot location in meters. Default is zero.
    :type position_y: float
    :param joint_names: Names of the joints
    :type joint_names: list
    :param delay: Optional time delay of first setpoint. Default is zero.
    :type delay: float
    """

    def __init__(
        self,
        duration,
        mid_point_fraction,
        starting_position,
        subgait_id,
        joint_names,
        position_x,
        position_y=0,
        delay=0,
    ):
        self.mid_point_fraction = mid_point_fraction
        self.delay = delay
        self.time = [
            self.delay,
            self.delay + self.mid_point_fraction * duration,
            self.delay + duration,
        ]
        self.starting_position = starting_position
        self.position_x = position_x
        self.position_y = position_y
        self.joint_names = joint_names
        self.subgait_id = subgait_id
        self.pose = Pose()

    def middle_setpoint(self):
        """Calls IK solver to compute the joint angles needed for the middle setpoint

        :returns: A setpoint_dict for the middle position.
        :rtype: dict
        """
        middle_position = self.pose.solve_mid_position(
            self.position_x, self.position_y, self.mid_point_fraction, self.subgait_id
        )

        self.middle_setpoint_dict = self.from_list_to_setpoint(
            self.joint_names,
            middle_position,
            None,
            self.time[1],
        )

    def desired_setpoint(self):
        """Calls IK solver to compute the joint angles needed for the desired x and y coordinate

        :param position_x: x-coordinate in meters of the desired foot location.
        :type position_x: float
        :param position_y: Optional y-coordinate in meters of the desired foot location. Default is zero.
        :type position_y: float
        """
        self.desired_position = self.pose.solve_end_position(
            self.position_x, self.position_y, self.subgait_id
        )

        self.desired_setpoint_dict = self.from_list_to_setpoint(
            self.joint_names, self.desired_position, None, self.time[2]
        )

    def get_final_position(self):
        """Get setpoint_dictionary of the final setpoint.

        :return: The final setpoint of the subgait.
        :rtype: dict
        """
        return self.from_list_to_setpoint(
            self.joint_names, self.desired_position, None, self.time[0]
        )

    def to_joint_trajectory_class(self):
        """Creates a list of DynamicJointTrajectories for each joint"""
        self.joint_trajectory_list = []
        for name in self.joint_names:
            self.joint_trajectory_list.append(
                DynamicJointTrajectory(
                    [
                        self.starting_position[name],
                        self.middle_setpoint_dict[name],
                        self.desired_setpoint_dict[name],
                    ]
                )
            )

    def to_joint_trajectory_msg(self):
        """Create joint_trajectory_msg containing the interpolated trajectories for each joint

        :returns: A joint_trajectory_msg
        :rtype: joint_trajectory_msg
        """
        # Update pose:
        pose_list = [joint.position for joint in self.starting_position.values()]
        self.pose = Pose(pose_list)

        # Solve for middle setpoint:
        self.middle_setpoint()

        # Solve for desired setpoint:
        self.desired_setpoint()

        # Create joint_trajectory_msg
        self.to_joint_trajectory_class()
        joint_trajectory_msg = trajectory_msg.JointTrajectory()
        joint_trajectory_msg.joint_names = self.joint_names

        timestamps = np.linspace(self.time[0], self.time[-1], 20)

        for timestamp in timestamps:
            joint_trajecory_point = trajectory_msg.JointTrajectoryPoint()
            joint_trajecory_point.time_from_start = Duration(timestamp).to_msg()

            for joint_trajectory in self.joint_trajectory_list:
                joint_trajectory.interpolate_setpoints()
                interpolated_setpoint = joint_trajectory.get_interpolated_setpoint(
                    timestamp
                )

                joint_trajecory_point.positions.append(interpolated_setpoint.position)
                joint_trajecory_point.velocities.append(interpolated_setpoint.velocity)

            joint_trajectory_msg.points.append(joint_trajecory_point)

        return joint_trajectory_msg

    def from_list_to_setpoint(self, joint_names, position, velocity, time):
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

        if velocity is not None:
            for i in range(len(joint_names)):
                setpoint_dict.update(
                    {
                        joint_names[i]: Setpoint(
                            Duration(time),
                            position[i],
                            velocity[i],
                        )
                    }
                )
        else:
            for i in range(len(joint_names)):
                setpoint_dict.update(
                    {
                        joint_names[i]: Setpoint(
                            Duration(time),
                            position[i],
                            0.0,
                        )
                    }
                )

        return setpoint_dict
