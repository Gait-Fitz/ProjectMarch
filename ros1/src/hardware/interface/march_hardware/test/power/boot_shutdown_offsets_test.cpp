/*
 * Copyright (C) 2021 Thijs Raymakers
 * Copyright (C) 2020 Olav de Haas
 * Copyright (C) 2019 Tim Buckers
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

// Copyright 2018 Project March.
#include "march_hardware/power/boot_shutdown_offsets.h"
#include "gtest/gtest.h"
#include <sstream>

class BootShutdownOffsetsTest : public ::testing::Test {
protected:
    const int masterOkByteOffset = 4;
    const int shutdownByteOffset = 3;
    const int shutdownAllowedByteOffset = 2;
};

TEST_F(BootShutdownOffsetsTest, ValidValues)
{
    BootShutdownOffsets bootShutdownOffsets(
        masterOkByteOffset, shutdownByteOffset, shutdownAllowedByteOffset);

    EXPECT_EQ(masterOkByteOffset, bootShutdownOffsets.getMasterOkByteOffset());
    EXPECT_EQ(shutdownByteOffset, bootShutdownOffsets.getShutdownByteOffset());
    EXPECT_EQ(shutdownAllowedByteOffset,
        bootShutdownOffsets.getShutdownAllowedByteOffset());
}
TEST_F(BootShutdownOffsetsTest, EmptyConstructor)
{
    BootShutdownOffsets bootShutdownOffsets;

    EXPECT_THROW(
        bootShutdownOffsets.getMasterOkByteOffset(), std::runtime_error);
    EXPECT_THROW(
        bootShutdownOffsets.getShutdownByteOffset(), std::runtime_error);
    EXPECT_THROW(
        bootShutdownOffsets.getShutdownAllowedByteOffset(), std::runtime_error);
}

TEST_F(BootShutdownOffsetsTest, InValidValues)
{
    EXPECT_THROW(BootShutdownOffsets bootShutdownOffsets(
                     masterOkByteOffset, -7, shutdownAllowedByteOffset),
        std::runtime_error);
    EXPECT_THROW(BootShutdownOffsets bootShutdownOffsets(
                     -1, shutdownByteOffset, shutdownAllowedByteOffset),
        std::runtime_error);
    EXPECT_THROW(BootShutdownOffsets bootShutdownOffsets(
                     masterOkByteOffset, shutdownByteOffset, -12),
        std::runtime_error);
}

TEST_F(BootShutdownOffsetsTest, Equals)
{
    BootShutdownOffsets bootShutdownOffsets1(
        masterOkByteOffset, shutdownByteOffset, shutdownAllowedByteOffset);
    BootShutdownOffsets bootShutdownOffsets2(
        masterOkByteOffset, shutdownByteOffset, shutdownAllowedByteOffset);

    EXPECT_TRUE(bootShutdownOffsets1 == bootShutdownOffsets2);
}
TEST_F(BootShutdownOffsetsTest, NotEquals)
{
    BootShutdownOffsets bootShutdownOffsets1(
        masterOkByteOffset, shutdownByteOffset, shutdownAllowedByteOffset);
    BootShutdownOffsets bootShutdownOffsets2(masterOkByteOffset,
        /*shutdownByteOffset=*/17, shutdownAllowedByteOffset);
    BootShutdownOffsets bootShutdownOffsets3(masterOkByteOffset,
        shutdownByteOffset, /*shutdownAllowedByteOffset=*/32);

    EXPECT_FALSE(bootShutdownOffsets1 == bootShutdownOffsets2);
    EXPECT_FALSE(bootShutdownOffsets1 == bootShutdownOffsets3);
    EXPECT_FALSE(bootShutdownOffsets2 == bootShutdownOffsets3);
}

TEST_F(BootShutdownOffsetsTest, TestStream)
{
    BootShutdownOffsets bootShutdownOffsets(
        masterOkByteOffset, shutdownByteOffset, shutdownAllowedByteOffset);
    std::stringstream ss;
    ss << "bootShutdownOffsets: " << bootShutdownOffsets;
    EXPECT_EQ("bootShutdownOffsets: BootShutdownOffset(masterOk: 4, shutdown: "
              "3, shutdownAllowed: 2)",
        ss.str());
}
