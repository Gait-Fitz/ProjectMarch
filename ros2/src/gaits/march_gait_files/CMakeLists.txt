#[[
 Copyright (C) 2021 Wolf Nederpel
 Copyright (C) 2020 Bas Volkers, Katja Schmahl, Thijs Raymakers

 This program is free software: you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 Version 3 as published by the Free Software Foundation WITH
 additional terms published by Project MARCH per section 7 of
 the GNU General Public License Version 3.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License INCLUDING the additional terms for
 more details.

 You should have received a copy of the GNU General Public License
 AND the additional terms along with this program. If not,
 see <https://projectmarch.nl/s/LICENSE> and
 <https://projectmarch.nl/s/LICENSE-ADDITIONAL-TERMS>.
]]

cmake_minimum_required(VERSION 3.5)
project(march_gait_files)

find_package(ament_cmake REQUIRED)

install(DIRECTORY airgait-v airgait-vi minimal simulation_only-vi
        test_versions-vi training-v training-vi test_joint_gaits
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
