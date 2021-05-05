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

// Copyrihgt 2018 Project March
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"

#include "march_safety/input_device_safety.hpp"
#include "march_safety/safety_handler.hpp"
#include "march_safety/safety_node.hpp"
#include "march_safety/safety_type.hpp"
#include "march_safety/temperature_safety.hpp"

#include "march_shared_msgs/msg/error.hpp"
#include "march_shared_msgs/msg/gait_instruction.hpp"
#include "march_shared_msgs/srv/get_param_string_list.hpp"

#include "march_utility/node_utils.hpp"

#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto safety_node = std::make_shared<SafetyNode>();

    // Add the input device and temperature safety handlers
    auto safety_handler = std::make_shared<SafetyHandler>(safety_node);
    safety_node->safety_list.push_back(
        std::make_unique<InputDeviceSafety>(safety_node, safety_handler));
    safety_node->safety_list.push_back(
        std::make_unique<TemperatureSafety>(safety_node, safety_handler));

    safety_node->start();

    rclcpp::spin(safety_node);
    rclcpp::shutdown();
    return 0;
}

SafetyNode::SafetyNode()
    : Node("safety_node", "march",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
            /*automatically_declare_parameters_from_overrides=*/true))
{
    joint_names = node_utils::get_joint_names(*this);
    RCLCPP_DEBUG(this->get_logger(), "Got joint names.");

    // Create an error publisher to notify the system (state machine) if
    // something is wrong
    error_publisher = this->create_publisher<march_shared_msgs::msg::Error>(
        "/march/error", 1000);

    // Create an instruction publisher to publish when a gait has to stop
    gait_instruction_publisher
        = this->create_publisher<march_shared_msgs::msg::GaitInstruction>(
            "/march/input_device/instruction", 1000);
}

/**
 * @brief Start the safety node
 * @details Starts a wall timer that calls the update function every 50ms.
 */
void SafetyNode::start()
{
    // Ensure that the safety node is updated every 50 ms
    timer = this->create_wall_timer(50ms, std::bind(&SafetyNode::update, this));
}

/**
 * @brief Update the safety listeners
 */
void SafetyNode::update()
{
    for (auto& i : safety_list) {
        i->update();
    }
}