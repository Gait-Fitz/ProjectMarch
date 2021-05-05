# Copyright (C) 2019 Rutger van Beek, Tim Buckers
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
bool high_voltage_enabled

# The current consumed by the power distribution board.
float32 power_distribution_board_current

# The total current of all high voltage nets combined.
float32 high_voltage_net_current

# The request of the power distribution board to shutdown the master.
bool master_shutdown_requested

LowVoltageNet[] low_voltage_nets
HighVoltageNet[] high_voltage_nets

