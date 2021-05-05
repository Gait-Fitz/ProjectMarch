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

//// Copyright 2019 Project March.
#include "rclcpp/rclcpp.hpp"

#include "march_safety/safety_handler.hpp"
#include "march_safety/safety_node.hpp"
#include <march_shared_msgs/msg/error.hpp>
#include <march_shared_msgs/msg/gait_instruction.hpp>

#include <string>
#include <utility>

SafetyHandler::SafetyHandler(std::shared_ptr<SafetyNode> node)
    : node_(std::move(node))
{
}

/**
 * @brief Publish an error message to the /march/error topic
 * @param message Error message
 * @param error_type Type of the error
 */
void SafetyHandler::publishErrorMessage(
    const std::string& message, int8_t error_type)
{
    ErrorMsg error_msg;
    std::ostringstream message_stream;
    error_msg.header.stamp = node_->get_clock()->now();
    error_msg.error_message = message;
    error_msg.type = error_type;
    node_->error_publisher->publish(error_msg);
}

/**
 * @brief Publish a GaitInstruction stop message to the
 * /march/input_device/instruction topic
 */
void SafetyHandler::publishStopMessage()
{
    GaitInstruction gait_instruction_msg;
    gait_instruction_msg.header.stamp = node_->get_clock()->now();
    ;
    gait_instruction_msg.type = GaitInstruction::STOP;
    node_->gait_instruction_publisher->publish(gait_instruction_msg);
}

/**
 * @brief Publish a fatal error message to the /march/error topic
 * @param message Error message
 */
void SafetyHandler::publishFatal(const std::string& message)
{
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());

    publishErrorMessage(message, ErrorMsg::FATAL);
}

/**
 * @brief Publish a non-fatal error message to the /march/error topic
 * @param message Error message
 */
void SafetyHandler::publishNonFatal(const std::string& message)
{
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());

    publishStopMessage();
    publishErrorMessage(message, ErrorMsg::NON_FATAL);
}
