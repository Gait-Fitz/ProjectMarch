# Copyright (C) 2021 Thijs Raymakers, Wolf Nederpel
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

from python_qt_binding.QtWidgets import QCheckBox, QPushButton
import rospy


class SideSubgaitView(object):
    def __init__(self, widget, side: str = None):
        if side == "previous":
            self.import_button = widget.findChildren(
                QPushButton, "import_previous_subgait_button"
            )[0]
            self.default_checkbox = widget.findChildren(
                QCheckBox, "previous_is_standing_check_box"
            )[0]
            self.lock_checkbox = widget.findChildren(
                QCheckBox, "lock_startpoint_check_box"
            )[0]
        elif side == "next":
            self.import_button = widget.findChildren(
                QPushButton, "import_next_subgait_button"
            )[0]
            self.default_checkbox = widget.findChildren(
                QCheckBox, "next_is_standing_check_box"
            )[0]
            self.lock_checkbox = widget.findChildren(
                QCheckBox, "lock_endpoint_check_box"
            )[0]
        else:
            rospy.loginfo(
                "SideSubgaitView initialized without specified side, "
                "can raise issues with locking setpoints."
            )
            self.import_button = widget.findChildren(QPushButton)[0]
            self.default_checkbox = widget.findChildren(QCheckBox)[1]
            self.lock_checkbox = widget.findChildren(QCheckBox)[0]

    def update_widget(self, controller):
        self.import_button.setText(controller.subgait_text)

        self.lock_checkbox.blockSignals(True)
        self.lock_checkbox.setChecked(controller.lock_checked)
        self.lock_checkbox.blockSignals(False)

        self.default_checkbox.blockSignals(True)
        self.default_checkbox.setChecked(controller.default_checked)
        self.default_checkbox.blockSignals(False)
