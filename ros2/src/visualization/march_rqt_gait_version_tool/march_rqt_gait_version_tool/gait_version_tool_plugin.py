# Copyright (C) 2021 Thijs Raymakers
# Copyright (C) 2020 Bas Volkers, Katja Schmahl
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

import os
import sys
from qt_gui.plugin import Plugin
import rclpy
from rclpy.node import Node
from .gait_version_tool_controller import GaitVersionToolController
from .gait_version_tool_view import GaitVersionToolView
from rqt_gui.main import Main
from ament_index_python import get_package_share_directory


def main(args=None):
    """The main function used to start up the rqt note taker."""
    rclpy.init(args=args)

    try:
        plugin = "march_rqt_gait_version_tool"
        main_plugin = Main(filename=plugin)
        sys.exit(main_plugin.main(standalone=plugin))

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


class GaitVersionToolPlugin(Plugin):
    def __init__(self, context):
        """Initiating the viewer and controller for the gait selection interface."""
        super(GaitVersionToolPlugin, self).__init__(context)
        self._node: Node = context.node
        self.setObjectName("GaitVersionToolPlugin")

        ui_file = os.path.join(
            get_package_share_directory("march_rqt_gait_version_tool"),
            "gait_selection.ui",
        )
        self._controller = GaitVersionToolController(self._node)
        self._widget = GaitVersionToolView(ui_file, self._controller)
        context.add_widget(self._widget)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                "{0} ({1})".format(self._widget.windowTitle(), context.serial_number())
            )
