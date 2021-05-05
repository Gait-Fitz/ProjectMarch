# Copyright (C) 2021 Thijs Raymakers
# Copyright (C) 2020 Katja Schmahl, Roel Vos
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

#!/usr/bin/env python
from .nosetests.gait_generator_controller_test import GaitGeneratorControllerTest
from .nosetests.modifiable_joint_trajectory_test import ModifiableJointTrajectoryTest
from .nosetests.modifiable_setpoint_test import ModifiableSetpointTest
from .nosetests.modifiable_subgait_test import ModifiableSubgaitTest
import rosunit

PKG = "march_rqt_gait_generator"

if __name__ == "__main__":
    rosunit.unitrun(PKG, "modifiable_setpoint_test", ModifiableSetpointTest)
    rosunit.unitrun(
        PKG, "modifiable_joint_trajectory_test", ModifiableJointTrajectoryTest
    )
    rosunit.unitrun(PKG, "modifiable_subgait_test", ModifiableSubgaitTest)
    rosunit.unitrun(PKG, "gait_generator_controller_test", GaitGeneratorControllerTest)
