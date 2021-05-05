/*
 * Copyright (C) 2021 Thijs Raymakers
 * Copyright (C) 2020 Olav de Haas
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
#include "march_hardware/ethercat/pdo_interface.h"
#include "march_hardware/ethercat/pdo_types.h"

#include <memory>

#include <gmock/gmock.h>

class MockPdoInterface : public march::PdoInterface {
public:
    MockPdoInterface() = default;

    MOCK_METHOD3(write8, void(uint16_t, uint8_t, march::bit8));
    MOCK_METHOD3(write16, void(uint16_t, uint8_t, march::bit16));
    MOCK_METHOD3(write32, void(uint16_t, uint8_t, march::bit32));

    MOCK_CONST_METHOD2(read8, march::bit8(uint16_t, uint8_t));
    MOCK_CONST_METHOD2(read16, march::bit16(uint16_t, uint8_t));
    MOCK_CONST_METHOD2(read32, march::bit32(uint16_t, uint8_t));
};

using MockPdoInterfacePtr = std::shared_ptr<MockPdoInterface>;
