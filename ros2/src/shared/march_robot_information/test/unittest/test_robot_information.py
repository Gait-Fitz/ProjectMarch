# Copyright (C) 2021 Bas Volkers, Thijs Raymakers
#
# This program is free software: you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# Version 3 as published by the Free Software Foundation WITH
# additional terms published by Project MARCH per section 7 of
# the GNU General Public License Version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License INCLUDING the additional terms for
# more details.
#
# You should have received a copy of the GNU General Public License
# AND the additional terms along with this program. If not,
# see <https://projectmarch.nl/s/LICENSE> and
# <https://projectmarch.nl/s/LICENSE-ADDITIONAL-TERMS>.

import unittest

import rclpy

from march_robot_information.robot_information_node import RobotInformation
from march_shared_msgs.srv import GetJointNames


class TestGaitSelection(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    def test_create_with_predefined_joint_names(self):
        joint_names = ["one", "two"]
        node = RobotInformation(joint_names=joint_names)
        request = GetJointNames.Request()
        response = GetJointNames.Response(joint_names=[])
        response = node.get_joint_names_cb(request, response)
        self.assertEqual(joint_names, response.joint_names)
