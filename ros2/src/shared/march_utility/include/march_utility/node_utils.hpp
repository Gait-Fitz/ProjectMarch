/*
 * Copyright (C) 2021 Bas Volkers, Thijs Raymakers
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

// Copyright 2020 Project March.
#ifndef MARCH_UTILITY_NODE_UTILS_HPP
#define MARCH_UTILITY_NODE_UTILS_HPP

#include "rclcpp/rclcpp.hpp"
#include <string>

namespace node_utils {
// Get the Joint names from the robot information node.
std::vector<std::string> get_joint_names(rclcpp::Node& node);
} // namespace node_utils

#endif // MARCH_UTILITY_NODE_UTILS_HPP
