import os
import unittest

from rclpy.time import Time
from ament_index_python import get_package_share_directory
from urdf_parser_py import urdf

from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait import (
    DynamicSetpointGait,
)
from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.gait_selection import GaitSelection

VALID_PACKAGE = "march_gait_selection"
VALID_DIRECTORY = "test/resources"


class TestDynamicSetpoint(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
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

    def setUp(self):
        self.gait = DynamicSetpointGait(self.gait_selection)

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
        first_subgait_delay = 0.2
        gait_update = self.gait.start(current_time, first_subgait_delay)

        self.assertEqual(
            gait_update,
            GaitUpdate.should_schedule_early(self.gait._get_trajectory_command),
            "GaitUpdate instance not correct",
        )
        start_time = current_time + first_subgait_delay
        self.assertEqual(
            self.gait._start_time,
            start_time,
            "Start time not equal to current_time + first_subgait_delay",
        )
        self.assertEqual(
            self.gait._end_time,
            Time(seconds=(first_subgait_delay + 1.5)),
            "End time not equal to 1.5 + first_subgait_delay",
        )
        self.assertFalse(
            self.gait._start_is_delayed, "_start_is_delayed bool not correct"
        )
