# Copyright (C) 2021 Thijs Raymakers
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

class GaitVersionToolError(Exception):
    def __init__(self, msg):
        super(GaitVersionToolError, self).__init__(msg)


class InvalidResponseError(GaitVersionToolError):
    def __init__(self):
        super(InvalidResponseError, self).__init__("Failed to parse response")


class GaitServiceError(GaitVersionToolError):
    def __init__(self):
        super(GaitServiceError, self).__init__("Failed to contact gait selection")
