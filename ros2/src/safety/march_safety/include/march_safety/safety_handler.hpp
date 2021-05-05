/*
 * Copyright (C) 2021 Bas Volkers, Thijs Raymakers
 * Copyright (C) 2020 Bas Volkers
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
#ifndef MARCH_SAFETY_SAFETY_HANDLER_H
#define MARCH_SAFETY_SAFETY_HANDLER_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "march_safety/safety_node.hpp"

#include <march_shared_msgs/msg/error.hpp>
#include <march_shared_msgs/msg/gait_instruction.hpp>

class SafetyHandler {
    using ErrorMsg = march_shared_msgs::msg::Error;
    using GaitInstruction = march_shared_msgs::msg::GaitInstruction;

public:
    explicit SafetyHandler(std::shared_ptr<SafetyNode> node);

    // Publish a fatal error message to the /march/error topic
    void publishFatal(const std::string& message);

    // Publish a fatal error message to the /march/error topic
    void publishNonFatal(const std::string& message);

    // Publish an error message to the /march/error topic
    void publishErrorMessage(const std::string& message, int8_t error_type);

    // Publish a GaitInstruction stop message to the
    // /march/input_device/instruction topic
    void publishStopMessage();

private:
    std::shared_ptr<SafetyNode> node_;
};

#endif // MARCH_SAFETY_SAFETY_HANDLER_H
