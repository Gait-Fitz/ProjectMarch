# Copyright (C) 2020 Roel Vos
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

from qt_gui.plugin import Plugin
from urdf_parser_py import urdf

from .gait_generator_controller import GaitGeneratorController
from .gait_generator_view import GaitGeneratorView


class GaitGeneratorPlugin(Plugin):
    def __init__(self, context):
        super(GaitGeneratorPlugin, self).__init__(context)

        self.view = GaitGeneratorView()
        context.add_widget(self.view)

        robot = urdf.Robot.from_parameter_server()
        self.controller = GaitGeneratorController(self.view, robot)

    def shutdown_plugin(self):
        self.controller.stop_time_slider_thread()
