"""Author: Marten Haitjema, MVII"""

from rclpy.node import Node
from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait import (
    DynamicSetpointGait,
)
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand


class DynamicSetpointGaitSingleStep(DynamicSetpointGait):
    """Single step gait based on dynamic setpoint gait

    :param gait_selection_node: the gait selection node
    :type gait_selection_node: Node
    """

    def __init__(self, gait_selection_node: Node):
        super().__init__(gait_selection_node)
        self.gait_name = "dynamic_walk_single_step"

    def _get_next_command(self) -> TrajectoryCommand:
        """Create the next command. Because it is a single step, this will
        always be a left_swing and a close gait.

        :returns: A TrajectoryCommand for the next subgait
        :rtype: TrajectoryCommand
        """
        self.subgait_id = "left_swing"
        if self._end:
            # If the gait has ended, the next command should be None
            return None
        else:
            self._end = True
            return self._get_trajectory_command(stop=True)