# Copyright (C) 2021 Thijs Raymakers
# Copyright (C) 2019 Isha Dijks, Olav de Haas
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

from pyqtgraph.Qt import QtCore
import rospy


class TimeSliderThread(QtCore.QThread):

    update_signal = QtCore.pyqtSignal(int)

    def __init__(self, current, playback_speed, max_time):
        QtCore.QThread.__init__(self)
        self.current = current
        self.playback_speed = playback_speed
        self.max = max_time
        self.allowed_to_run = True

    def run(self):
        index = 0
        calculations_per_second = 30
        r = rospy.Rate(calculations_per_second)
        while self.allowed_to_run:
            index += 1
            value = (
                self.current
                + float(index) / calculations_per_second * self.playback_speed
            ) % self.max
            self.update_signal.emit(value)
            r.sleep()

    def stop(self):
        self.allowed_to_run = False
