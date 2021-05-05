/*
 * Copyright (C) 2021 Thijs Raymakers
 * Copyright (C) 2020 Olav de Haas, Rutger van Beek
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
#include <march_hardware_builder/hardware_builder.h>
#include <march_hardware_builder/hardware_config_exceptions.h>

#include <string>

#include <gtest/gtest.h>
#include <ros/package.h>

#include <march_hardware/encoder/incremental_encoder.h>

class IncrementalEncoderBuilderTest : public ::testing::Test {
protected:
    std::string base_path;

    void SetUp() override
    {
        this->base_path = ros::package::getPath("march_hardware_builder")
                              .append(/*__s=*/"/test/yaml/encoder");
    }

    YAML::Node loadTestYaml(const std::string& relative_path)
    {
        return YAML::LoadFile(this->base_path.append(relative_path));
    }
};

TEST_F(IncrementalEncoderBuilderTest, ValidIncrementalEncoder)
{
    YAML::Node config = this->loadTestYaml("/incremental_encoder_correct.yaml");

    march::IncrementalEncoder expected = march::IncrementalEncoder(
        /*number_of_bits=*/12, /*transmission=*/45.5);
    auto created = HardwareBuilder::createIncrementalEncoder(config);
    ASSERT_EQ(expected, *created);
}

TEST_F(IncrementalEncoderBuilderTest, NoConfig)
{
    YAML::Node config;
    ASSERT_EQ(nullptr, HardwareBuilder::createIncrementalEncoder(config[""]));
}

TEST_F(IncrementalEncoderBuilderTest, NoResolution)
{
    YAML::Node config
        = this->loadTestYaml("/incremental_encoder_no_resolution.yaml");

    ASSERT_THROW(
        HardwareBuilder::createIncrementalEncoder(config), MissingKeyException);
}

TEST_F(IncrementalEncoderBuilderTest, NoTransmission)
{
    YAML::Node config
        = this->loadTestYaml("/incremental_encoder_no_transmission.yaml");

    ASSERT_THROW(
        HardwareBuilder::createIncrementalEncoder(config), MissingKeyException);
}
