"""Author: Marten Haitjema, MVII."""

import os
import yaml

from queue import Queue
from typing import Dict, Optional
from ament_index_python import get_package_share_path
from rclpy.node import Node

from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait_step_and_close import (
    DynamicSetpointGaitStepAndClose,
)
from march_gait_selection.dynamic_interpolation.dynamic_subgait import DynamicSubgait
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_utility.utilities.utility_functions import get_position_from_yaml
from march_utility.utilities.logger import Logger

from march_shared_msgs.msg import FootPosition
from geometry_msgs.msg import Point
from std_msgs.msg import Header


class DynamicSetpointGaitStepAndHold(DynamicSetpointGaitStepAndClose):
    """Class for stepping to a hold position firstly, and to the desired position secondly."""

    _use_position_queue: bool

    def __init__(self, gait_selection_node: Node):
        self.subgait_id = "right_swing"
        self._end_position_right = {}
        self._end_position_left = {}
        super().__init__(gait_selection_node)
        self.logger = Logger(gait_selection_node, __class__.__name__)
        self.gait_name = "dynamic_step_and_hold"

        self.update_parameter()
        if self._use_position_queue:
            self._create_position_queue()

        self._end_position_right = get_position_from_yaml("stand")
        self._end_position_right["right_hip_aa"] = 0
        self._end_position_right["left_hip_aa"] = 0
        self._end_position_right["right_hip_fe"] = 0
        self._end_position_right["left_hip_fe"] = 0
        self._end_position_right["right_knee"] = 1

        self._end_position_left = get_position_from_yaml("stand")
        self._end_position_left["right_hip_aa"] = 0
        self._end_position_left["left_hip_aa"] = 0
        self._end_position_left["right_hip_fe"] = 0
        self._end_position_left["left_hip_fe"] = 0
        self._end_position_left["left_knee"] = 1

    def _reset(self) -> None:
        """Reset all attributes of the gait."""
        self._should_stop = False
        self._end = False

        self._start_time_next_command = None
        self._current_time = None

        self._next_command = None

        self._start_is_delayed = True
        self._scheduled_early = False

        if self.start_position_all_joints == self._end_position_right:
            self.subgait_id = "right_swing"
        elif self.start_position_all_joints == self._end_position_left:
            self.subgait_id = "left_swing"

    def _create_subgait_instance(
        self,
        start_position: Dict[str, float],
        subgait_id: str,
        start: bool,
        stop: bool,
    ) -> DynamicSubgait:
        """Create a DynamicSubgait instance.

        Args:
            start_position (Dict[str, float]): dict containing joint_names and positions of the joint as floats
            subgait_id (str): either 'left_swing' or 'right_swing'
            start (bool): whether it is a start gait or not
            stop (bool): whether it is a stop gait or not
        Returns:
            DynamicSubgait: DynamicSubgait instance for the desired step
        """
        if subgait_id == "right_swing":
            end_position = self._end_position_right
        else:
            end_position = self._end_position_left

        return DynamicSubgait(
            self.gait_selection,
            end_position,
            start_position,
            subgait_id,
            self.joint_names,
            self.foot_location,
            self.joint_soft_limits,
            start,
            stop,
        )

    def _get_trajectory_command(self, start=False, stop=False) -> Optional[TrajectoryCommand]:
        """Return a TrajectoryCommand based on current subgait_id, or based on the _position_queue if enabled.

        Args:
            start (Optional[bool]): whether it is a start gait or not, default False
            stop (Optional[bool]): whether it is a stop gait or not, default False
        Returns:
            TrajectoryCommand: command with the current subgait and start time
        """
        if stop:
            self.logger.info("Stopping dynamic gait.")
            self.foot_location = FootPosition(processed_point=Point(x=0.0, y=0.0, z=0.44699999999999995), duration=1.5)
        else:
            if self._use_position_queue:
                if not self.position_queue.empty():
                    self.foot_location = self._get_foot_location_from_queue()
                else:
                    return None
            else:
                try:
                    self.foot_location = self._get_foot_location(self.subgait_id)
                    stop = self._check_msg_time(self.foot_location)
                except AttributeError:
                    self.logger.info("No FootLocation found. Connect the camera or use simulated points.")
                    self._end = True
                    return None

            self.logger.warn(
                f"Stepping to location ({self.foot_location.processed_point.x}, "
                f"{self.foot_location.processed_point.y}, {self.foot_location.processed_point.z})"
            )

        return self._get_first_feasible_trajectory(start, stop)

    def _get_foot_location_from_queue(self) -> FootPosition:
        """Get FootPosition message from the position queue.

        Returns:
            FootPosition: FootPosition msg with position from queue
        """
        header = Header(stamp=self.gait_selection.get_clock().now().to_msg())
        point_from_queue = self.position_queue.get()
        point = Point(x=point_from_queue["x"], y=point_from_queue["y"], z=point_from_queue["z"])

        if self.position_queue.empty():
            self.logger.warn("Position queue empty. Use force unknown + homestand to close the gait.")

        return FootPosition(header=header, processed_point=point, duration=self.duration_from_yaml)

    def update_parameter(self) -> None:
        """Updates '_use_position_queue' to the newest value in gait_selection."""
        self._use_position_queue = self.gait_selection.use_position_queue
        if self._use_position_queue:
            self._create_position_queue()

    def _create_position_queue(self) -> None:
        """Creates and fills the queue with values from position_queue.yaml."""
        queue_path = get_package_share_path("march_gait_selection")
        queue_directory = os.path.join(queue_path, "position_queue", "position_queue.yaml")
        with open(queue_directory, "r") as queue_file:
            position_queue_yaml = yaml.load(queue_file, Loader=yaml.SafeLoader)

        self.duration_from_yaml = position_queue_yaml["duration"]
        self.points_from_yaml = position_queue_yaml["points"]
        self.position_queue = Queue()
        self._fill_queue()

    def _fill_queue(self) -> None:
        """Fills the position queue with the values specified in position_queue.yaml."""
        if self._use_position_queue:
            for point in self.points_from_yaml:
                self.position_queue.put(point)

    def _add_point_to_queue(self, point: Point) -> None:
        """Adds a point to the end of the queue.

        Args:
            point (Point): point message to add to the queue.
        """
        point_dict = {"x": point.x, "y": point.y, "z": point.z}
        self.position_queue.put(point_dict)
        self.logger.info(f"Point added to position queue. Current queue is: {list(self.position_queue.queue)}")