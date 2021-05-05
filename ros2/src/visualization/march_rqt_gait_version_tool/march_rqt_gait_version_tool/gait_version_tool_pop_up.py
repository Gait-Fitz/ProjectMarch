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

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QDialog,
    QGridLayout,
    QLabel,
    QScrollArea,
    QSizePolicy,
    QWidget,
)


class PopUpWindow(QDialog):
    def __init__(self, parent, width=500, height=600):
        """Base class for creating a pop up window over an existing widget.

        :param parent:
            The parent widget to connect to the pop up
        :param width:
            Starting width of the the pop up widget
        :param height:
            Starting height of the the pop up widget
        """
        super(PopUpWindow, self).__init__(parent=parent, flags=Qt.Window)
        self.resize(width, height)
        self.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)

        self._layout = QGridLayout(self)

        self._scroll_area = QScrollArea()
        self._scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self._scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self._scroll_area.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        self._scroll_area.setWidgetResizable(True)

        self._content_frame = QWidget(self._scroll_area, flags=Qt.Window)
        self._content_frame.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        self._content = QGridLayout(self._content_frame)

        self.msg_label = QLabel(self)
        self._content.addWidget(self.msg_label)

        self._scroll_area.setWidget(self._content_frame)
        self._layout.addWidget(self._scroll_area)

    def show_message(self, message):
        """Add message to the pop up and show the window."""
        self.msg_label.clear()
        self.msg_label.setText(message)
        return super(PopUpWindow, self).show()
