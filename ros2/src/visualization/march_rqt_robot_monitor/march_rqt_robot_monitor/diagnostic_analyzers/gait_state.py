# Copyright (C) 2021 Bas Volkers, Thijs Raymakers
# Copyright (C) 2020 Bas Volkers, Joris Weeda, Olav de Haas
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

"""The module gait_state.py contains the CheckGaitStatus Class."""

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
from rclpy.node import Node

from march_shared_msgs.msg import CurrentGait


class CheckGaitStatus(object):
    """Base class to diagnose the gait status."""

    def __init__(self, node: Node, updater: Updater):
        """Initializes an gait diagnostic which analyzes gait and subgait states.

        :type updater: diagnostic_updater.Updater
        """
        self._goal_sub = node.create_subscription(
            topic="/march/gait_selection/current_gait",
            msg_type=CurrentGait,
            callback=self._cb_goal,
            qos_profile=10,
        )
        self._gait_msg = None

        updater.add("Gait", self._diagnostics)

    def _cb_goal(self, msg: CurrentGait):
        """Set the current_gait.

        Callback for the gait scheduler goal.
        """
        self._gait_msg = msg

    def _diagnostics(self, stat):
        """Create a diagnostic function corresponding to gait and subgait data."""
        if self._gait_msg is None:
            stat.add("Topic error", "No events recorded")
            stat.summary(DiagnosticStatus.STALE, "No gait activity recorded")
            return stat

        stat.add("Gait", str(self._gait_msg.gait))
        stat.add("Subgait", str(self._gait_msg.subgait))
        stat.add("Subgait version", str(self._gait_msg.version))
        stat.add("Gait type", str(self._gait_msg.gait_type))

        stat.summary(
            DiagnosticStatus.OK,
            f"Gait: {self._gait_msg.gait}, {self._gait_msg.subgait}",
        )

        return stat
