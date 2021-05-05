# Copyright (C) 2021 Thijs Raymakers
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

import math

from python_qt_binding.QtWidgets import QTableWidgetItem

from .joint_setting_spin_box_delegate import JointSettingSpinBoxDelegate
from .model.modifiable_setpoint import ModifiableSetpoint


class JointTableController(object):
    TABLE_DIGITS = 4

    def __init__(self, joint_table_widget, joint):
        self.table_widget = joint_table_widget
        self.update_setpoints(joint)

    def update_setpoints(self, joint):
        self.table_widget.setRowCount(len(joint.setpoints))

        for i, setpoint in enumerate(joint.setpoints):
            time_item = QTableWidgetItem(str(round(setpoint.time, self.TABLE_DIGITS)))

            position_item = QTableWidgetItem(
                str(round(math.degrees(setpoint.position), self.TABLE_DIGITS))
            )

            velocity_item = QTableWidgetItem(
                str(round(math.degrees(setpoint.velocity), self.TABLE_DIGITS))
            )

            self.table_widget.setItem(i, 0, time_item)
            self.table_widget.setItem(i, 1, position_item)
            self.table_widget.setItem(i, 2, velocity_item)

        self.table_widget.setItemDelegate(
            JointSettingSpinBoxDelegate(
                joint.limits.velocity,
                joint.limits.lower,
                joint.limits.upper,
                joint.duration,
            )
        )
        self.table_widget.resizeColumnsToContents()

    def to_setpoints(self):
        setpoints = []
        for i in range(0, self.table_widget.rowCount()):
            time = float(self.table_widget.item(i, 0).text())
            position = math.radians(float(self.table_widget.item(i, 1).text()))
            velocity = math.radians(float(self.table_widget.item(i, 2).text()))
            setpoints.append(ModifiableSetpoint(time, position, velocity))
        return setpoints
