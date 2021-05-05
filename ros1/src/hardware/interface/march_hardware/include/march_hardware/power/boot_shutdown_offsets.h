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

// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_BOOT_SHUTDOWN_OFFSETS_H
#define MARCH_HARDWARE_BOOT_SHUTDOWN_OFFSETS_H

#include <ros/ros.h>

class BootShutdownOffsets {
    int masterOk;
    int shutdown;
    int shutdownAllowed;

public:
    BootShutdownOffsets(int masterOkByteOffset, int shutdownByteOffset,
        int shutdownAllowedByteOffset)
        : masterOk(masterOkByteOffset)
        , shutdown(shutdownByteOffset)
        , shutdownAllowed(shutdownAllowedByteOffset)
    {
        if (masterOkByteOffset < 0 || shutdownByteOffset < 0
            || shutdownAllowedByteOffset < 0) {
            ROS_ERROR("Negative byte offset not possible");
            throw std::runtime_error("Negative byte offset not possible");
        }
    }

    BootShutdownOffsets()
    {
        masterOk = -1;
        shutdown = -1;
        shutdownAllowed = -1;
    }

    int getMasterOkByteOffset() const
    {
        if (masterOk == -1) {
            ROS_FATAL("masterOkOffset is -1");
            throw std::runtime_error("masterOkOffset is -1");
        }
        return masterOk;
    }

    int getShutdownByteOffset() const
    {
        if (shutdown == -1) {
            ROS_FATAL("shutdownOffset is -1");
            throw std::runtime_error("shutdownOffset is -1");
        }
        return shutdown;
    }

    int getShutdownAllowedByteOffset() const
    {
        if (shutdownAllowed == -1) {
            ROS_FATAL("shutdownAllowedOffset is -1");
            throw std::runtime_error("shutdownAllowedOffset is -1");
        }
        return shutdownAllowed;
    }

    /** @brief Override comparison operator */
    friend bool operator==(
        const BootShutdownOffsets& lhs, const BootShutdownOffsets& rhs)
    {
        return lhs.masterOk == rhs.masterOk && lhs.shutdown == rhs.shutdown
            && lhs.shutdownAllowed == rhs.shutdownAllowed;
    }

    /** @brief Override stream operator for clean printing */
    friend ::std::ostream& operator<<(
        std::ostream& os, const BootShutdownOffsets& bootShutdownOffsets)
    {
        return os << "BootShutdownOffset(masterOk: "
                  << bootShutdownOffsets.masterOk << ", "
                  << "shutdown: " << bootShutdownOffsets.shutdown << ", "
                  << "shutdownAllowed: " << bootShutdownOffsets.shutdownAllowed
                  << ")";
    }
};

#endif // MARCH_HARDWARE_BOOT_SHUTDOWN_OFFSETS_H
