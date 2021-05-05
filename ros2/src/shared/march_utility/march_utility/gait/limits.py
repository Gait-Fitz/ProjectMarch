# Copyright (C) 2021 Thijs Raymakers
# Copyright (C) 2020 Olav de Haas, Roel Vos, Rutger van Beek
# Copyright (C) 2019 Roel Vos
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

class Limits(object):
    def __init__(
        self, lower, upper, velocity, effort=None, k_position=None, k_velocity=None
    ):
        self.lower = lower
        self.upper = upper
        self.velocity = velocity
        self.effort = effort
        self.k_position = k_position
        self.k_velocity = k_velocity

    @classmethod
    def from_urdf_joint(cls, urdf_joint):
        """Creates limits from a given URDF joint."""
        return cls(
            urdf_joint.safety_controller.soft_lower_limit,
            urdf_joint.safety_controller.soft_upper_limit,
            urdf_joint.limit.velocity,
            urdf_joint.limit.effort,
            urdf_joint.safety_controller.k_position,
            urdf_joint.safety_controller.k_velocity,
        )

    def __eq__(self, other):
        return (
            self.lower == other.lower
            and self.upper == other.upper
            and self.velocity == other.velocity
            and self.effort == other.effort
            and self.k_position == other.k_position
            and self.k_velocity == other.k_velocity
        )

    def __ne__(self, other):
        return not self == other
