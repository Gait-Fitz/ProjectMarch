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

from typing import Optional

from attr import dataclass
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand


@dataclass
class GaitUpdate:
    """The GaitUpdate is a data structure for the different gaits to communicate with
    the gait state machine."""

    new_trajectory_command: Optional[TrajectoryCommand]
    is_new_subgait: bool
    is_finished: bool

    @staticmethod
    def empty():
        """A GaitUpdate with no information.

        Use when no action should be taken.
        """
        return GaitUpdate(None, False, False)

    @staticmethod
    def finished():
        """A GaitUpdate with the is_finished flag set to true.

        Use when the current gait should be stopped.
        """
        return GaitUpdate(None, False, True)

    @staticmethod
    def subgait_updated():
        """A GaitUpdate with the is_new_subgait flag set to true.

        Use when the TrajectoryScheduler should not schedule a new TrajectoryCommand, but the current subgait should be updated.
        """
        return GaitUpdate(None, True, False)

    @staticmethod
    def should_schedule(command):
        """A GaitUpdate that contains a new TrajectoryCommand, and also the is_new_subgait flag set to true indicating that the subgait has updated.

        Use when the TrajectoryScheduler should schedule a new TrajectoryCommand, and the current subgait should not be updated.
        """
        return GaitUpdate(command, True, False)

    @staticmethod
    def should_schedule_early(command: TrajectoryCommand):
        """A GaitUpdate that contains a new TrajectoryCommand, but the current subgait is not be updated yet

        Use when the TrajectoryScheduler should schedule a new TrajectoryCommand, but the current subgait should not be updated.
        """
        return GaitUpdate(command, False, False)
