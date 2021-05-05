# Copyright (C) 2021 Thijs Raymakers
# Copyright (C) 2020 Cilia Claij, Olav de Haas
# Copyright (C) 2019 Olav de Haas, Rogier Krijnen
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

import rospy

from .dynamic_pid_reconfigurer import DynamicPIDReconfigurer


def main():
    rospy.init_node("march_gain_scheduling_node")

    while not rospy.is_shutdown() and not rospy.has_param("/march/joint_names"):
        rospy.sleep(0.5)
        rospy.logdebug("Waiting on /march/joint_names to be available")

    if rospy.is_shutdown():
        return

    joint_list = rospy.get_param("/march/joint_names")
    DynamicPIDReconfigurer(joint_list)

    rospy.spin()
