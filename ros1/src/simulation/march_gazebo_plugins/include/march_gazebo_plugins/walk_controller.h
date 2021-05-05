/*
 * Copyright (C) 2021 Katja Schmahl, Thijs Raymakers
 * Copyright (C) 2020 Katja Schmahl, Roel Vos
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
#ifndef MARCH_GAZEBO_PLUGINS_WALK_CONTROLLER_H
#define MARCH_GAZEBO_PLUGINS_WALK_CONTROLLER_H

#include <march_gazebo_plugins/obstacle_controller.h>
#include <ros/ros.h>

namespace gazebo {
class WalkController : public ObstacleController {
public:
    explicit WalkController(physics::ModelPtr model);
};
} // namespace gazebo

#endif // MARCH_GAZEBO_PLUGINS_WALK_CONTROLLER_H
