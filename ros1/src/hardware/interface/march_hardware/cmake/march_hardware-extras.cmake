# Copyright (C) 2020 Olav de Haas
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

# CMake function to add rpath to target
# Copied from https://github.com/shadow-robot/ros_ethercat/blob/kinetic-devel/ros_ethercat_hardware/cmake/ros_ethercat_hardware-extras.cmake.em
function(ros_enable_rpath target)
   # Set ${target} with RPATH built in so that we can install it suid
   set_target_properties(${target} PROPERTIES SKIP_BUILD_RPATH FALSE)

   # Set the install RPATH to the install path
   set(RPATH "${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}")

   # If LD_LIBRARY_PATH is set, add it to the install RPATH
   #  this works in a normal catkin environment, but fails if the user unsets
   #  their LD_LIBRARY_PATH manually for some reason
   if(DEFINED ENV{LD_LIBRARY_PATH})
      set(RPATH "${RPATH}:$ENV{LD_LIBRARY_PATH}")
   endif()

   # Apply our computed RPATH to the target
   set_target_properties(${target} PROPERTIES INSTALL_RPATH ${RPATH})

   # Don't use the final RPATH in devel space
   set_target_properties(${target} PROPERTIES BUILD_WITH_INSTALL_RPATH FALSE)
endfunction()
