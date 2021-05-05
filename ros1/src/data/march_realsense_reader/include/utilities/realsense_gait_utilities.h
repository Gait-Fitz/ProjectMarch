/*
 * Copyright (C) 2021 Katja Schmahl, Thijs Raymakers
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * Version 3 as published by the Free Software Foundation WITH
 * additional terms published by Project MARCH per section 7 of
 * the GNU General Public License Version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License INCLUDING the additional terms for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * AND the additional terms along with this program. If not,
 * see <https://projectmarch.nl/s/LICENSE> and
 * <https://projectmarch.nl/s/LICENSE-ADDITIONAL-TERMS>.
 */

#ifndef MARCH_REALSENSE_GAIT_UTILITIES_H
#define MARCH_REALSENSE_GAIT_UTILITIES_H

/** This enum is used for specifying the obstacle that should be dynamically
 * made with the RealSense camera, made to match the
 *  GetGaitParametersRealSense.srv
 */
enum SelectedGait {
    stairs_up = 0,
    stairs_down = 1,
    ramp_up = 2,
    ramp_down = 3
};

#endif // MARCH_REALSENSE_GAIT_UTILITIES_H
