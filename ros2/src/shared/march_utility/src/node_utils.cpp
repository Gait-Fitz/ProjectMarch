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
#include "march_utility/node_utils.hpp"
#include "march_shared_msgs/srv/get_joint_names.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace node_utils {
/*
 * @brief Get the Joint names from the robot information node.
 * @param node Node to create client for and use logger of.
 * @return Returns the list of joint names.
 */
std::vector<std::string> get_joint_names(rclcpp::Node& node)
{
    std::vector<std::string> names;

    auto client = node.create_client<march_shared_msgs::srv::GetJointNames>(
        "/march/robot_information/get_joint_names");
    auto request
        = std::make_shared<march_shared_msgs::srv::GetJointNames::Request>();

    // Wait until the service is available. Sending a request to an unavailable
    // service will always fail.
    while (!client->wait_for_service(5s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node.get_logger(),
                "Interrupted while waiting for get_joint_names service. "
                "Exiting.");
            return names;
        }
        RCLCPP_INFO(node.get_logger(),
            "The get_joint_names service was unavailable, waiting again...");
    }

    // Send the request and push the received names to the names vector.
    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(
            node.get_node_base_interface(), result)
        == rclcpp::FutureReturnCode::SUCCESS) {
        names = std::move(result.get()->joint_names);
    } else {
        RCLCPP_ERROR(
            node.get_logger(), "Failed to call get_joint_names service");
    }

    return names;
}
} // namespace node_utils
