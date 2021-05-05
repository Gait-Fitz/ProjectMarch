# Copyright (C) 2021 Bas Volkers
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

"""The module updater.py contains the DiagnosticUpdater Node."""

import diagnostic_updater
import rclpy
from march_utility.utilities.node_utils import get_joint_names
from rclpy.node import Node
from sensor_msgs.msg import JointState

from march_shared_msgs.msg import Alive

from .diagnostic_analyzers.check_input_device import CheckInputDevice
from .diagnostic_analyzers.control import CheckJointValues
from .diagnostic_analyzers.gait_state import CheckGaitStatus
from .diagnostic_analyzers.imc_state import CheckImcStatus

NODE_NAME = "rqt_robot_monitor"
HARDWARE_ID = "MARCH VI"

PERIOD = 0.1


class DiagnosticUpdater(Node):
    """This node uses the diagnostic analyzers to provide data for the diagnostic aggregator node."""

    def __init__(self):
        super().__init__(NODE_NAME)

        self.updater = diagnostic_updater.Updater(node=self, period=PERIOD)
        self.updater.setHardwareID(HARDWARE_ID)

        joint_names = get_joint_names(self)

        # Frequency checks
        CheckInputDevice(self, "/march/input_device/alive", Alive, self.updater, 4)

        # Control checks
        check_joint_states = CheckJointValues(self, "/march/joint_states", JointState)
        self.updater.add(
            "Control position values", check_joint_states.position_diagnostics
        )
        self.updater.add(
            "Control velocity values", check_joint_states.velocity_diagnostics
        )
        self.updater.add("Control effort values", check_joint_states.effort_diagnostics)

        # IMC state check
        CheckImcStatus(self, self.updater, joint_names)

        # Gait information
        CheckGaitStatus(self, self.updater)

    def update(self):
        """Update the DiagnosticUpdater if there are more than 0 tasks."""
        if len(self.updater.tasks) > 0:
            self.updater.update()

    def start(self):
        """Start the update timer."""
        self.create_timer(PERIOD, self.update)


def main():
    """Start the DiagnosticUpdater Node and the update timer."""
    rclpy.init()

    node = DiagnosticUpdater()
    node.start()

    rclpy.spin(node)
