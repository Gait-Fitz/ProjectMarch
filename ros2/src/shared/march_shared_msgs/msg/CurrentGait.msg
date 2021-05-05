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

std_msgs/Header header
# Gait name that is executing
string gait
# Subgait name that is executing, if the gait has subgaits, else empty string
string subgait
# Optional version name of the gait/subgait
string version
# Optional duration of the gait/subgait
builtin_interfaces/Duration duration
# Optional gait type name of the gait/subgait
string gait_type
