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

from PyQt5.QtGui import QColor, QPainter, QPixmap
from PyQt5.QtWidgets import QAbstractButton


class ImageButton(QAbstractButton):
    """
    A class to define the standard button on the input device
    """

    def __init__(self, image_path):
        super(ImageButton, self).__init__()
        self._pixmap = QPixmap(image_path)

    def paintEvent(self, event):  # noqa: N802
        painter = QPainter(self)
        painter.drawPixmap(0, 0, self._pixmap)
        if not self.isEnabled():
            painter.setOpacity(0.3)
            painter.fillRect(event.rect(), QColor(0, 0, 0))
