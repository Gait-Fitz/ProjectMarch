# Copyright (C) 2021 Bas Volkers, Katja Schmahl
# Copyright (C) 2020 Katja Schmahl
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

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .gait_selection import GaitSelection
from march_gait_selection.state_machine.gait_state_machine import GaitStateMachine
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryScheduler


def main():
    """ Starts up the gait selection node with the state machine and scheduler. """
    rclpy.init()

    gait_selection = GaitSelection()
    scheduler = TrajectoryScheduler(gait_selection)
    gait_state_machine = GaitStateMachine(gait_selection, scheduler)
    gait_state_machine.run()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(gait_selection, executor)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
