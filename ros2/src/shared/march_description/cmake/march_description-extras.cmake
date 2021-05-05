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

# build_and_install_xacro_files(<file> [<file> ...])
#
# Builds urdf from given xacro files and installs them into devel
# and install space. The urdf will be generated in the binary build
# dir of cmake and then installed into urdf/. For example, an xacro
# file named urdf/robot.xacro will be installed to urdf/robot.urdf.
function(build_and_install_xacro_files)
    foreach(input ${ARGN})
        if(${input} MATCHES "(.*)[.]xacro$")
            # Match path without xacro extension
            set(path ${CMAKE_MATCH_1})
            # Get the name of the xacro file without directories
            get_filename_component(name ${path} NAME)

            xacro_add_files(${path}.xacro
                TARGET ${name}_xacro
                REMAP build:=true
                OUTPUT ${CMAKE_BINARY_DIR}/${name}.urdf
            )

            install(FILES ${CMAKE_BINARY_DIR}/${name}.urdf
                DESTINATION share/${PROJECT_NAME}/urdf
            )

        endif()
    endforeach()
endfunction()
