# Copyright (C) 2021 Katja Schmahl
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

"""Executable to start the moveit interface node."""
import rospy
from march_shared_msgs.srv import (
    GetMoveItTrajectory,
)
from march_moveit_interface.moveit_interface import MoveItInterface


def main():
    rospy.init_node("balance_gaits")

    moveit_interface = MoveItInterface()

    rospy.Service(
        "/march/moveit/get_trajectory",
        GetMoveItTrajectory,
        moveit_interface.get_joint_trajectory,
    )

    rospy.spin()
