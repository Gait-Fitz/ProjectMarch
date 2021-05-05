/*
 * Copyright (C) 2021 Maarten ten Voorde, Thijs Raymakers
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

#ifndef MARCH_MPC_REFERENCES_H
#define MARCH_MPC_REFERENCES_H

#include <vector>

using namespace std;

/**
 * \brief scroll/progress the reference vector
 * @param reference
 */
void scrollReference(vector<vector<double>>& reference);

/**
 * \brief Add a sinus reference to the existing reference vector with the
 * following parameters
 * @param reference
 * @param freq
 * @param amplitude
 * @param N
 * @param dt
 */
void sinRef(vector<vector<double>>& reference, double freq, double amplitude,
    int N, double dt);

/**
 * \brief Add a step reference to the existing reference vector with the
 * following parameters
 * @param reference
 * @param amplitude
 * @param N
 */
void stepRef(vector<vector<double>>& reference, double amplitude, int N);

#endif // MARCH_MPC_REFERENCES_H
