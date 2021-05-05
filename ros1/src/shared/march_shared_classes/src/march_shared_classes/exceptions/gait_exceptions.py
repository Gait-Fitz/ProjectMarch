# Copyright (C) 2021 Thijs Raymakers
# Copyright (C) 2020 Joris Weeda, Olav de Haas, Rutger van Beek,
#                    Wolf Nederpel
# Copyright (C) 2019 Joris Weeda
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

class GaitError(Exception):
    def __init__(self, msg=None):
        """Base class for exceptions in gait modules.

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = "An error occurred with a gait module."
        super(GaitError, self).__init__(msg)


class GaitNameNotFound(GaitError):
    def __init__(self, gait_name, msg=None):
        """Class to raise an error when given gait name does not exists .

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = "Could not find gait name: {gait} in map.".format(gait=gait_name)

        super(GaitNameNotFound, self).__init__(msg)


class SubgaitNameNotFound(GaitError):
    def __init__(self, subgait_name, gait_name, msg=None):
        """Class to raise an error when given subgait name does not exists .

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = "Could not find subgait name {subgait} of gait {gait} in map.".format(
                subgait=subgait_name, gait=gait_name
            )

        super(SubgaitNameNotFound, self).__init__(msg)


class NonValidGaitContent(GaitError):
    def __init__(self, gait_name=None, msg=None):
        """Class to raise an error when given gait has incorrect content .

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = "The given gait: {gn} has incorrect information".format(gn=gait_name)

        super(NonValidGaitContent, self).__init__(msg)


class SubgaitGraphError(GaitError):
    def __init__(self, msg):
        super(SubgaitGraphError, self).__init__(msg)


class TransitionError(GaitError):
    def __init__(self, msg=None):
        """Class to raise an error when transition between two subgaits has an error .

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = "Subgaits can not transition"

        super(TransitionError, self).__init__(msg)


class SubgaitInterpolationError(GaitError):
    def __init__(self, msg=None):
        """Class to raise an error when it was not possible to interpolate between subgaits."""
        if msg is None:
            msg = "An error occurred while trying to merge two subgaits."

        super(SubgaitInterpolationError, self).__init__(msg)
