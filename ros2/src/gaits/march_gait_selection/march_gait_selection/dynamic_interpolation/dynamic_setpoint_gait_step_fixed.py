"""Author: Marten Haitjema."""

from rclpy.node import Node
from typing import Optional
from march_shared_msgs.msg import FootPosition
from march_utility.utilities.logger import Logger
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH
from .dynamic_setpoint_gait_step import DynamicSetpointGaitStep


class DynamicSetpointGaitStepFixed(DynamicSetpointGaitStep):
    """Step gait based on dynamic setpoint gait, with a fixed step size.

    Args:
        gait_selection_node (GaitSelection): the gait selection node.

    Attributes:
        _fixed_foot_location (FootPosition): the foot location to step to, set in gait_preprocessor.
    """

    def __init__(self, gait_selection_node: Node):
        super().__init__(gait_selection_node)
        self._logger = Logger(gait_selection_node, __class__.__name__)
        self.gait_name = "fixed_step"
        gait_selection_node.create_subscription(
            FootPosition,
            "/march/fixed_foot_position",
            self._update_foot_location,
            DEFAULT_HISTORY_DEPTH,
        )

    def _update_foot_location(self, foot_position: FootPosition) -> None:
        """Update left and right foot position."""
        self._fixed_foot_location = foot_position

    def _get_foot_location(self, subgait_id: str) -> Optional[FootPosition]:
        """Return the fixed foot location."""
        return self._fixed_foot_location
