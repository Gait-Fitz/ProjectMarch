"""Author: Marten Haitjema, MVII"""

import os
import unittest
import rclpy

from rclpy.time import Time
from ament_index_python import get_package_share_directory
from urdf_parser_py import urdf

from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait import (
    DynamicSetpointGait,
)
from march_gait_selection.gait_selection import GaitSelection
from march_utility.utilities.duration import Duration

from geometry_msgs.msg import Point

VALID_PACKAGE = "march_gait_selection"
VALID_DIRECTORY = "test/resources"


class TestDynamicSetpointGait(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.robot = urdf.Robot.from_xml_file(
            get_package_share_directory("march_description") + "/urdf/march6.urdf"
        )
        cls.gait_directory = os.path.join(
            get_package_share_directory(VALID_PACKAGE), VALID_DIRECTORY
        )
        cls.gait_selection = GaitSelection(
            gait_package=VALID_PACKAGE,
            directory=VALID_DIRECTORY,
            robot=cls.robot,
            balance=False,
            dynamic_gait=True,
        )

        # Set parameters manually, because parameters server is not online
        cls.gait_selection.dynamic_subgait_duration = 1.5
        cls.gait_selection.middle_point_fraction = 0.45
        cls.gait_selection.middle_point_height = 0.15
        cls.gait_selection.minimum_stair_height = 0.15

    def setUp(self):
        self.gait = DynamicSetpointGait(self.gait_selection)

        # Set parameters manually, because parameters server is not online
        point = Point()
        point.x = 0.4
        point.y = 0.0
        point.z = 0.0
        self.gait.foot_position_right = point
        self.gait.foot_position_left = point

    def test_init_start_position(self) -> None:
        """Assert that the dynamic gait starts in the home stand position"""
        homestand_joint_dict = {
            "left_ankle": 0.0,
            "left_hip_aa": 0.0349,
            "left_hip_fe": -0.1745,
            "left_knee": 0.0,
            "right_ankle": 0.0,
            "right_hip_aa": 0.0349,
            "right_hip_fe": -0.1745,
            "right_knee": 0.0,
        }
        homestand_setpoint_dict = self.gait._joint_dict_to_setpoint_dict(
            homestand_joint_dict
        )
        self.assertEqual(
            self.gait.start_position,
            homestand_setpoint_dict,
            "Dynamic gait should start in homestand position",
        )

    def test_start_method(self) -> None:
        """Assert that the start method returns a correct TrajectoryCommand"""
        current_time = Time(seconds=0)
        first_subgait_delay = Duration(0.2)
        self.gait.start(current_time, first_subgait_delay)

        start_time = current_time + first_subgait_delay
        self.assertEqual(
            self.gait._start_time,
            start_time,
            "Start time not equal to current_time + first_subgait_delay",
        )
        self.assertEqual(
            self.gait._end_time,
            Time(seconds=1.7),
            "End time not equal to 1.5 + first_subgait_delay",
        )
        self.assertFalse(
            self.gait._start_is_delayed, "_start_is_delayed bool not correct"
        )
