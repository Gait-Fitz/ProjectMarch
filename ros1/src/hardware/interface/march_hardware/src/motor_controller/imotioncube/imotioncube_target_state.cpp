/*
 * Copyright (C) 2021 Bas Volkers, Thijs Raymakers
 * Copyright (C) 2019 Isha Dijks
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

// Copyright 2019 Project March.

#include <march_hardware/motor_controller/imotioncube/imotioncube_target_state.h>

namespace march {
const IMotionCubeTargetState IMotionCubeTargetState::SWITCH_ON_DISABLED
    = IMotionCubeTargetState("Switch on Disabled", /*controlWord=*/128,
        /*stateMask=*/0b0000000001001111, /*state=*/64);
const IMotionCubeTargetState IMotionCubeTargetState::READY_TO_SWITCH_ON
    = IMotionCubeTargetState("Ready to Switch On", /*controlWord=*/6,
        /*stateMask=*/0b0000000001101111, /*state=*/33);
const IMotionCubeTargetState IMotionCubeTargetState::SWITCHED_ON
    = IMotionCubeTargetState("Switched On", /*controlWord=*/7,
        /*stateMask=*/0b0000000001101111, /*state=*/35);
const IMotionCubeTargetState IMotionCubeTargetState::OPERATION_ENABLED
    = IMotionCubeTargetState("Operation Enabled", /*controlWord=*/15,
        /*stateMask=*/0b0000000001101111, /*state=*/39);
} // namespace march
