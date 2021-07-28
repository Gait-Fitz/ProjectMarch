import unittest
import rclpy
from ament_index_python import get_package_share_directory, PackageNotFoundError
from march_shared_msgs.srv import ContainsGait
from urdf_parser_py import urdf
from march_gait_selection.gait_selection import GaitSelection
from march_gait_selection.state_machine.gait_interface import GaitInterface
from march_utility.gait.gait import Gait

VALID_PACKAGE = "march_gait_selection"
VALID_DIRECTORY = "test/testing_gait_files"


class TestGaitSelection(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.robot = urdf.Robot.from_xml_file(
            get_package_share_directory("march_description") + "/urdf/march6.urdf"
        )

    def setUp(self):
        self.gait_selection = GaitSelection(
            gait_package=VALID_PACKAGE, directory=VALID_DIRECTORY, robot=self.robot
        )
        self.gait_selection.get_logger().warn(
            f"gait_directory called with {VALID_DIRECTORY}"
        )

    # __init__ tests
    def test_init_with_wrong_package(self):
        with self.assertRaises(PackageNotFoundError):
            GaitSelection(gait_package="wrong", directory=VALID_DIRECTORY)

    def test_init_with_wrong_directory(self):
        with self.assertRaises(FileNotFoundError):
            GaitSelection(gait_package=VALID_PACKAGE, directory="wrong")

    # load gaits tests
    def test_types_in_loaded_gaits(self):
        for gait in self.gait_selection._gaits.values():
            self.assertIsInstance(gait, GaitInterface)

    def test_gait_selection_positions(self):
        self.assertIn("stand", self.gait_selection.positions)

    # scan directory tests
    def test_scan_directory_top_level_content(self):
        directory = self.gait_selection.scan_directory()
        directory_gaits = [
            "walk_medium",
            "realsense_stand",
            "realsense_sit",
            "balance_walk",
            "stairs_up",
            "walk_small",
            "walk",
            "walk_single_step_from_walk",
            "walk_single_step_from_walk_other_directory",
        ]
        self.assertEqual(sorted(directory.keys()), sorted(directory_gaits))

    def test_scan_directory_subgait_versions(self):
        directory = self.gait_selection.scan_directory()
        self.assertEqual(directory["walk"]["left_swing"], ["MV_walk_leftswing_v2"])

    # get item tests
    def test_get_item_with_wrong_name(self):
        self.assertIsNone(self.gait_selection["wrong"])

    def test_get_item_type(self):
        self.assertIsInstance(self.gait_selection["walk"], Gait)

    def test_contains_gait_true(self):
        request = ContainsGait.Request(gait="walk", subgaits=["right_open"])
        response = ContainsGait.Response(contains=False)
        response = self.gait_selection.contains_gait_cb(request, response)
        self.assertTrue(response.contains)

    def test_contains_gait_wrong_subgait(self):
        request = ContainsGait.Request(gait="walk", subgaits=["non_existing_subgait"])
        response = ContainsGait.Response(contains=True)
        response = self.gait_selection.contains_gait_cb(request, response)
        self.assertFalse(response.contains)

    def test_contains_gait_wrong_gait(self):
        request = ContainsGait.Request(
            gait="non_existing_gait", subgaits=["right_open"]
        )
        response = ContainsGait.Response(contains=True)
        response = self.gait_selection.contains_gait_cb(request, response)
        self.assertFalse(response.contains)

    def test_gait_name_reading_elsewhere(self):
        self.assertEqual(
            self.gait_selection._gaits["walk_single_step_from_walk"].gait_name,
            "walk_single_step_from_walk",
        )

    def test_subgaits_reading_elsewhere(self):
        self.assertEqual(
            set(self.gait_selection._gaits["walk_single_step_from_walk"].subgaits),
            {"right_open", "left_close"},
        )

    def test_subgait_name_reading_other_gait(self):
        self.assertEqual(
            self.gait_selection._gaits["walk_single_step_from_walk"]
            .subgaits["right_open"]
            .version,
            "MV_walk_rightopen_v2",
        )

    def test_subgait_name_reading_other_gait_and_directory(self):
        self.assertEqual(
            self.gait_selection._gaits["walk_single_step_from_walk_other_directory"]
            .subgaits["right_open"]
            .version,
            "MV_walk_rightopen_other_directory_v2",
        )
