/*
 * Copyright (C) 2021 Thijs Raymakers
 * Copyright (C) 2020 Thijs Raymakers
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

#include "march_fake_sensor_data/FakeTemperatureData.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <vector>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Create a FakeTemperatureNode instance.
    std::vector<float> autoregression_weights { 0.1, 0.1, 0.1, 0.15, 0.15, 0.2,
        0.2 };
    auto temperature = std::make_shared<FakeTemperatureDataNode>(
        "march_fake_temperature_data", std::move(autoregression_weights));
    // Start the timer and create the publishers
    (*temperature).initialize();

    // Execute the temperature node with a single-threaded executor.
    rclcpp::spin(temperature);
    rclcpp::shutdown();
    return 0;
}
