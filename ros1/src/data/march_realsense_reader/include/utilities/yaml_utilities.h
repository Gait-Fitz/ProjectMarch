/*
 * Copyright (C) 2021 Thijs Raymakers, Wolf Nederpel
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

#ifndef MARCH_YAML_UTILITIES_H
#define MARCH_YAML_UTILITIES_H
#include "yaml-cpp/yaml.h"
#include <optional>
#include <ros/ros.h>
#include <string>
#include <typeinfo>

namespace yaml_utilities {
// Grab a parameter from a YAML::Node and throws a clear warning if the
// requested parameter does not exist
template <typename T>
std::optional<T> grabParameter(
    YAML::Node const& yaml_node, std::string const& parameter_name)
{
    T parameter;
    if (YAML::Node raw_parameter = yaml_node[parameter_name]) {
        parameter = raw_parameter.as<T>();
        return parameter;
    } else {
        ROS_ERROR_STREAM(
            "Parameter not found in the given YAML::node. Parameter name is "
            << parameter_name);
        return std::nullopt;
    }
}
} // namespace yaml_utilities

#endif // MARCH_YAML_UTILITIES_H
