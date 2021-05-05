# Copyright (C) 2021 Thijs Raymakers
# Copyright (C) 2020 Roel Vos, Wolf Nederpel
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

from march_rqt_gait_generator.model.modifiable_setpoint import ModifiableSetpoint


class ModifiableSetpointTest(unittest.TestCase):
    def setUp(self):
        self.setpoint = ModifiableSetpoint(1.123412541, 0.034341255, 123.162084549)

    def test_invert_time(self):
        self.setpoint.invert(2)
        self.assertAlmostEqual(self.setpoint.time, 2 - 1.123412541, 8)

    def test_invert_position(self):
        self.setpoint.invert(2)
        self.assertAlmostEqual(self.setpoint.position, 0.034341255, 8)

    def test_invert_velocity(self):
        self.setpoint.invert(2)
        self.assertAlmostEqual(self.setpoint.velocity, -123.162084549, 8)
