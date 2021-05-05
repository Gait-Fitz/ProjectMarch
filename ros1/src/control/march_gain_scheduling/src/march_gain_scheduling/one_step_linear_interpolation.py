# Copyright (C) 2021 Thijs Raymakers
# Copyright (C) 2019 Cilia Claij, Olav de Haas
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

from .interpolation_errors import NegativeValueError, UnequalLengthError


def interpolate(current_gains, needed_gains, gradient, delta_t):
    if len(current_gains) != len(needed_gains):
        raise UnequalLengthError(
            "current_gains and needed_gains do not have the same length"
        )
    if gradient <= 0 or delta_t < 0:
        raise NegativeValueError("gradient or delta_t are below zero")
    next_gains = [0] * len(current_gains)
    for i in range(len(current_gains)):
        if current_gains[i] > needed_gains[i]:
            next_gains[i] = max(needed_gains[i], current_gains[i] - gradient * delta_t)
        else:
            next_gains[i] = min(needed_gains[i], current_gains[i] + gradient * delta_t)
    return next_gains
