from rclpy.time import Time
from typing import Optional

from march_utility.gait.edge_position import EdgePosition, StaticEdgePosition
from march_utility.utilities.duration import Duration
from march_utility.utilities.utility_functions import get_joint_names_from_urdf
from march_utility.gait.setpoint import Setpoint

from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.state_machine.gait_interface import GaitInterface
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_gait_selection.dynamic_interpolation.dynamic_subgait import DynamicSubgait


class DynamicSetpointGait(GaitInterface):
    """Gait built up from dynamic setpoints"""

    def __init__(self):
        super(DynamicSetpointGait, self).__init__()
        self._should_stop = False
        self._end = False
        self._is_transitioning = False

        self._start_time = None
        self._end_time = None
        self._current_time = None

        self._current_command = None
        self._next_command = None

        self.start_position = {
            "left_ankle": Setpoint(Duration(0), 0.0, 0),
            "left_hip_aa": Setpoint(Duration(0), 0.0349, 0),
            "left_hip_fe": Setpoint(Duration(0), -0.1745, 0),
            "left_knee": Setpoint(Duration(0), 0.0, 0),
            "right_ankle": Setpoint(Duration(0), 0.0, 0),
            "right_hip_aa": Setpoint(Duration(0), 0.0349, 0),
            "right_hip_fe": Setpoint(Duration(0), -0.1745, 0),
            "right_knee": Setpoint(Duration(0), 0.0, 0),
        }
        self.end_position = self.start_position

        self.joint_names = get_joint_names_from_urdf()
        self.gait_name = "dynamic_walk"

        self._start_is_delayed = False
        self._scheduled_early = False

    @property
    def name(self):
        return self.gait_name

    @property
    def subgait_name(self):
        # Should return left_swing/right_swing for simulation to work
        return self.subgait_id

    @property
    def version(self):
        return ""

    @property
    def duration(self):
        if self._current_command is not None:
            return self._current_command.duration
        else:
            return None

    @property
    def gait_type(self):
        # For now only take walk-like gaits
        if self._current_command is not None:
            return "walk_like"
        else:
            return None

    @property
    def starting_position(self) -> EdgePosition:
        return StaticEdgePosition(self.setpoint_dict_to_joint_dict(self.start_position))

    @property
    def final_position(self) -> EdgePosition:
        if self._current_command is not None:
            return StaticEdgePosition(
                self.setpoint_dict_to_joint_dict(
                    self.dynamic_subgait.get_final_position()
                )
            )
        else:
            return StaticEdgePosition(
                self.setpoint_dict_to_joint_dict(self.end_position)
            )

    @property
    def subsequent_subgaits_can_be_scheduled_early(self) -> bool:
        return True

    @property
    def first_subgait_can_be_scheduled_early(self) -> bool:
        return True

    def _reset(self):
        """Reset all attributes of the gait"""
        self._should_stop = False
        self._end = False
        self._is_transitioning = False

        self._start_time = None
        self._end_time = None
        self._current_time = None

        self._current_command = None
        self._next_command = None

        self._start_is_delayed = False
        self._scheduled_early = False

    DEFAULT_FIRST_SUBGAIT_START_DELAY = Duration(0)

    def start(
        self,
        current_time: Time,
        first_subgait_delay: Optional[Duration] = DEFAULT_FIRST_SUBGAIT_START_DELAY,
    ) -> GaitUpdate:
        """Starts the gait. If first_subgait_delay is not zero, the first subgait will
        be scheduled with the given delay.

        :param current_time: Time at which the subgait will start
        :type current_time: Time
        :param first_subgait_delay: Delay of first subgait schedule
        :type first_subgait_delay: Duration

        :return: A GaitUpdate containing a TrajectoryCommand
        :rtype: GaitUpdate
        """
        self._reset()
        self._current_time = current_time
        self.subgait_id = "right_swing"
        self._first_subgait_delay = first_subgait_delay

        if first_subgait_delay > Duration(0):
            self._start_is_delayed = True
            self._start_time = self._current_time + first_subgait_delay
        else:
            self._start_is_delayed = False
            self._start_time = self._current_time

        self._current_command = self.subgait_id_to_trajectory_command()

        if self._start_is_delayed:
            return GaitUpdate.should_schedule_early(self._current_command)
        else:
            return GaitUpdate.should_schedule(self._current_command)

    DEFAULT_EARLY_SCHEDULE_UPDATE_DURATION = Duration(0)

    def update(
        self,
        current_time: Time,
        early_schedule_duration: Optional[
            Duration
        ] = DEFAULT_EARLY_SCHEDULE_UPDATE_DURATION,
    ) -> GaitUpdate:
        """Give an update on the progress of the gait.

        If the subgaitgait is finished, schedule the next subgait. Else,
        return an empty GaitUdpate. If early_schedule_duration is not zero,
        the next subgait will be planned ahead with the given duration.

        :param current_time: Current time.
        :type current_time: Time
        :param early_schedule_duration: Duration that determines how long ahead the next subgait is planned
        :type early_schedule_duration: Duration

        :return: GaitUpdate containing TrajectoryCommand when finished, else empty GaitUpdate
        :rtype: GaitUpdate
        """
        self._current_time = current_time

        if self._start_is_delayed:
            if self._current_time >= self._start_time:
                self._start_is_delayed = False
                # Update start position and time stamps for the next gait after
                # the start gait has been scheduled
                self.update_start_pos()
                self._update_time_stamps(self._next_command.duration)
                return GaitUpdate.subgait_updated()
            else:
                return GaitUpdate.empty()

        if self._current_time >= self._end_time:
            # The current subgait has reached its end time, a new subgait should
            # be scheduled
            return self._update_next_subgait()

        if (
            early_schedule_duration > Duration(0)
            and not self._scheduled_early
            and self._current_time >= self._end_time - early_schedule_duration
        ):
            # Schedule the next subgait early, if this has not been done yet
            return self._update_next_subgait_early()

        return GaitUpdate.empty()

    def _update_next_subgait(self) -> GaitUpdate:
        """Update the next subgait.

        If the current subgait is left_swing, the next subgait should be
        right_swing, and vice versa. Also checks if the next subgait should
        be scheduled early.

        :return: A GaitUpdate containg a TrajectoryCommand
        :rtype: GaitUpdate
        """
        if self._scheduled_early:
            # We scheduled early and already determined the next subgait
            next_command = self._next_command
        else:
            next_command = self._get_next_command()

        if next_command is None:
            # The stop gait has completed and the gait is finished
            return GaitUpdate.finished()

        # Update the starting position and time stamps for the next command
        self.update_start_pos()
        self._update_time_stamps(next_command.duration)

        if not self._scheduled_early:
            return GaitUpdate.should_schedule(next_command)
        else:
            # Reset early schedule attributes
            self._scheduled_early = False
            self._next_command = None
            return GaitUpdate.subgait_updated()

    def _update_next_subgait_early(self) -> GaitUpdate:
        """Updates the next subgait early."""
        self._scheduled_early = True
        self._next_command = self._get_next_command()

        if self._next_command is None:
            return GaitUpdate.empty()

        return GaitUpdate.should_schedule_early(self._next_command)

    def _get_next_command(self):
        """Create the next command, based on what the current subgait is.
        Also checks if the gait has to be stopped. If true, it returns
        a close gait.

        :returns: A TrajectoryCommand for the next subgait
        :rtype: TrajectoryCommand
        """

        if self.subgait_id == "right_swing":
            self.subgait_id = "left_swing"
        elif self.subgait_id == "left_swing":
            self.subgait_id = "right_swing"

        if self._end:
            # If the gait is stopped, the next command should be None
            return None
        elif self._should_stop:
            return self.subgait_id_to_trajectory_command(stop=True)
        else:
            return self.subgait_id_to_trajectory_command()

    def stop(self) -> bool:
        """Called when the current gait should be stopped"""
        self._should_stop = True
        return True

    def end(self):
        """Called when the gait is finished"""
        self._current_command = None

    def update_start_pos(self):
        """Update the start position of the next subgait to be
        the last position of the previous subgait."""
        self.start_position = self.dynamic_subgait.get_final_position()

    def setpoint_dict_to_joint_dict(self, setpoint_dict):
        """Creates a joint_dict from a setpoint_dict.

        :param setpoint_dict: A setpoint_dictionary containing joint names and setpoints.
        :type setpoint_dict: dict

        :returns: A joint_dict containing joint names and positions.
        :rtype: dict
        """
        position = []
        for setpoint in setpoint_dict:
            position.append(setpoint_dict[setpoint].position)

        jointdict = {}
        for i in range(len(position)):
            jointdict.update({self.joint_names[i]: position[i]})

        return jointdict

    def subgait_id_to_trajectory_command(self, stop=False) -> TrajectoryCommand:
        """Construct a TrajectoryCommand from the current subgait_id

        :return: TrajectoryCommand with the current subgait and start time.
        :rtype: TrajectoryCommand
        """
        # Should be replaced by covid topic in the future
        if stop:
            desired_ankle_x = 0  # m
            desired_ankle_y = 0  # m
            subgait_duration = 1  # s
            self._end = True
        else:
            desired_ankle_x = 0.20  # m
            desired_ankle_y = 0.03  # m
            subgait_duration = 1.5  # s
        mid_point_frac = 0.45

        self.dynamic_subgait = DynamicSubgait(
            subgait_duration,
            mid_point_frac,
            self.start_position,
            self.subgait_id,
            self.joint_names,
            desired_ankle_x,
            position_y=desired_ankle_y,
        )

        trajectory = self.dynamic_subgait.to_joint_trajectory_msg()
        if self._start_is_delayed:
            self._end_time = self._start_time

        return TrajectoryCommand(
            trajectory,
            Duration(subgait_duration),
            self.subgait_id,
            self._end_time,
        )

    DEFAULT_FIRST_SUBGAIT_UPDATE_TIMESTAMPS_DELAY = Duration(0)

    def _update_time_stamps(self, next_command_duration):
        """Update the starting and end time

        :param next_command_duration: Duration of the next command to be scheduled.
        :type next_command_duration: Duration
        """
        self._start_time = self._end_time
        self._end_time = self._start_time + next_command_duration