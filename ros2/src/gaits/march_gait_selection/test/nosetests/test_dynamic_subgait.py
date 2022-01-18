"""Author: Marten Haitjema, MVII"""

import unittest

from march_gait_selection.dynamic_interpolation.dynamic_subgait import DynamicSubgait
from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait import DynamicSetpointGait

from geometry_msgs.msg import Point


class TestDynamicSubgait(unittest.TestCase):
    def setUp(self):
        self.gait = DynamicSetpointGait()
        self.subgait = DynamicSubgait(
            duration=1.5,
            middle_point_fraction=0.45,
            middle_point_height=0.15,
            starting_position=self.gait.start_position,
            subgait_id="right_swing",
            joint_names=self.gait.joint_names,
            position=Point(0.4, 0.0, 0.0),
            stop=False,
        )
