# Copyright (C) 2021 Bas Volkers, Thijs Raymakers
# Copyright (C) 2020 Roel Vos, Rutger van Beek
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

from march_utility.gait.limits import Limits


class LimitsTest(unittest.TestCase):
    def setUp(self):
        self.limits = Limits(0, 1, 2, 3, 4, 5)

    def test_lower_limit(self):
        self.assertEqual(self.limits.lower, 0, "Lower limit not initialised correctly.")

    def test_upper_limit(self):
        self.assertEqual(self.limits.upper, 1, "Upper limit not initialised correctly.")

    def test_velocity_limit(self):
        self.assertEqual(
            self.limits.velocity, 2, "Velocity limit not initialised correctly."
        )

    def test_effort(self):
        self.assertEqual(
            self.limits.effort, 3, "Effort limit not initialised correctly."
        )

    def test_k_position(self):
        self.assertEqual(
            self.limits.k_position, 4, "k_position not initialised correctly."
        )

    def test_k_velocity(self):
        self.assertEqual(
            self.limits.k_velocity, 5, "k_velocity not initialised correctly."
        )

    def test_equal_operator(self):
        self.assertEqual(self.limits, Limits(0, 1, 2, 3, 4, 5))

    def test_unequal_operator_1(self):
        # first entry different
        self.assertNotEqual(self.limits, Limits(1, 1, 2, 3, 4, 5))

    def test_unequal_operator_2(self):
        # second entry different
        self.assertNotEqual(self.limits, Limits(0, 2, 2, 3, 4, 5))

    def test_unequal_operator_3(self):
        # third entry different
        self.assertNotEqual(self.limits, Limits(0, 1, 3, 3, 4, 5))

    def test_unequal_operator_4(self):
        # fourth entry different
        self.assertNotEqual(self.limits, Limits(0, 1, 2, 4, 4, 5))

    def test_unequal_operator_5(self):
        # fifth entry different
        self.assertNotEqual(self.limits, Limits(0, 1, 2, 3, 5, 5))

    def test_unequal_operator(self):
        # sixth entry different
        self.assertNotEqual(self.limits, Limits(0, 1, 2, 3, 4, 6))
