# Copyright (C) 2021 Bas Volkers, Katja Schmahl, Thijs Raymakers
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

"""This module contains some errors that are specific to the Project March code."""
from __future__ import annotations
from march_utility.utilities.side import Side


class PackageNotFoundError(Exception):
    """Class to raise an error when a ros package cannot be found."""

    def __init__(self, package_name: str, msg: str = None) -> None:
        """
        Initialize PackageNotFoundError.

        :param package_name: The package that was not found.
        :param msg: Optional, a custom error message to return.
        """
        if msg is None:
            msg = f"Package: {package_name} could not be found."

        super(PackageNotFoundError, self).__init__(msg)


class MsgTypeError(Exception):
    """Class to raise an error when an non msg type is added to a message."""

    def __init__(self, msg: str = None) -> None:
        """
        Initialize MsgTypeError.

        :param msg: Optional, a custom error message to return.
        """
        if msg is None:
            msg = "A non msg type (defined in shared resources) was added to a ROS-message"

        super(MsgTypeError, self).__init__(msg)


class SideSpecificationError(Exception):
    """Class to raise an error when wrong side ('right' or 'left') was specified."""

    def __init__(self, foot: Side, msg: str = None) -> None:
        """
        Initialize side specification error.

        :param foot: The foot which was wrongly specified.
        :param msg: Optional, a custom error message to return.
        """
        if msg is None:
            msg = (
                f"An incorrect side was supplied. Must be a either Side.left or "
                f"Side.right, but was {foot}."
            )

        super(SideSpecificationError, self).__init__(msg)


class IncorrectCoordinateError(Exception):
    """Class to raise an error when the coordinates of a position are incorrect."""

    def __init__(self, msg: str = None) -> None:
        """
        Initialize IncorrectCoordinateError.

        :param msg: Optional, a custom error message to return.
        """
        if msg is None:
            msg = (
                "The keys of a position or velocity dictionary should be "
                "['x', 'y', 'z'], but were different."
            )

        super(IncorrectCoordinateError, self).__init__(msg)


class WeightedAverageError(Exception):
    """Class to raise an error when a weighted average cannot be computed."""

    def __init__(self, msg: str = None) -> None:
        """
        Initialize WeightedAverageError.

        :param msg: Optional, a custom error message to return.
        """
        if msg is None:
            msg = "The calculation of the weighted average cannot be executed safely."

        super(WeightedAverageError, self).__init__(msg)
