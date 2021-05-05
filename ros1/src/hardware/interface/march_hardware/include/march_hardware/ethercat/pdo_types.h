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

// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_ETHERCAT_PDO_TYPES_H
#define MARCH_HARDWARE_ETHERCAT_PDO_TYPES_H
#include <cstdint>

namespace march {
// struct used to easily get specific bytes from a 32 bit variable
struct packed_bit32 {
    uint8_t b0;
    uint8_t b1;
    uint8_t b2;
    uint8_t b3;
};

// union used for int32, uint32 and float
union bit32 {
    int32_t i;
    uint32_t ui;
    float f;
    packed_bit32 p;
};

// struct used to easily get specific bytes from a 16 bit variable
struct packed_bit16 {
    uint8_t b0;
    uint8_t b1;
};

// union used for int16 and uint16 in combination with the above struct
union bit16 {
    int16_t i;
    uint16_t ui;
    packed_bit16 p;
};

// union used for int8 and uint8 in combination with the above struct, uint8_t
// is used to read single byte unbiased
union bit8 {
    int8_t i;
    uint8_t ui;
    uint8_t b0;
};
} // namespace march
#endif // MARCH_HARDWARE_ETHERCAT_PDO_TYPES_H
