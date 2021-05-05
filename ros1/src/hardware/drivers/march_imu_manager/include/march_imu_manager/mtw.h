/*
 * Copyright (C) 2021 Thijs Raymakers
 * Copyright (C) 2019 Olav de Haas
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

#pragma once

#include <deque>
#include <mutex>

#include <ros/ros.h>

#include <xsensdeviceapi.h>
#include <xsenstypes.h>

class Mtw : public XsCallback {
public:
    explicit Mtw(XsDevice* device, size_t max_buffer_size = 100);

    /**
     * Returns whether any new packets are available.
     */
    bool dataAvailable();

    /**
     * Returns the oldest packet received from the MTw.
     * Does not delete it.
     */
    const XsDataPacket* getOldestPacket();

    /**
     * Deletes the oldest packet received from the MTw.
     */
    void deleteOldestPacket();

    /**
     * Returns the device id of the MTw. This id can also be found on the
     * back of the device.
     */
    const XsDeviceId getId() const;

protected:
    /**
     * Callback when new packets are available. This runs in a seperate
     * thread and only stores the packets in the buffer.
     */
    void onLiveDataAvailable(
        XsDevice* device, const XsDataPacket* packet) override final;

private:
    /**
     * Configures the MTw to output orientation, velocity and acceleration.
     */
    void configure();

    std::mutex mutex_;

    XsDevice* device_;

    size_t max_buffer_size_;
    std::deque<XsDataPacket> packet_buffer_;
};
