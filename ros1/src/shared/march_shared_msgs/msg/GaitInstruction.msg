# Copyright (C) 2020 Joris Weeda, Olav de Haas
# Copyright (C) 2019 Olav de Haas, Rutger van Beek, Tim Buckers
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

Header header

# The type of instruction.
int8 type

# All possible instruction types.
int8 UNKNOWN = -1
int8 STOP = 0
int8 GAIT = 1
int8 PAUSE = 2
int8 CONTINUE = 3
int8 INCREMENT_STEP_SIZE = 4
int8 DECREMENT_STEP_SIZE = 5

# If the type is a GAIT this is the name of the gait.
string gait_name

# Unique identifier for the input device sending this message. See `Alive`.
string id
