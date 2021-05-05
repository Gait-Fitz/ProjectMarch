/*
 * Copyright (C) 2021 Bas Volkers, Maarten ten Voorde, Thijs Raymakers,
 *                    Thijs Veen
 * Copyright (C) 2020 Maarten ten Voorde
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

#ifndef MARCH_MODEL_PREDICTIVE_CONTROLLER_H
#define MARCH_MODEL_PREDICTIVE_CONTROLLER_H

#include <acado_auxiliary_functions.h>
#include <iostream>
#include <vector>

using namespace std;

class ModelPredictiveController {

public:
    explicit ModelPredictiveController(std::vector<float> W);

    // Public variables
    vector<double> x0 { 0, 0 }; // Current state
    double u {}; // Calculated control input
    std::string joint_name;
    double cost {}; // Objective value

    // Timing variables
    acado_timer t {};
    double t_preparation {}, t_feedback {};

    // status variables
    int preparationStepStatus {};
    int feedbackStepStatus {};

    /**
     * \brief Initialise the model predictive controller
     */
    void init();

    /**
     * \brief Set the initial state
     * @param x0 - initial state
     */
    void setInitialState(vector<double> x0);

    /**
     * \brief Set the reference for time step n in [0, N]
     * @param n
     * @param reference
     */
    void setReference(int n, const std::vector<double>& reference);

    /**
     * \brief Assign the weighting array values
     * @param W - weighting array
     */
    void assignWeightingMatrix(std::vector<float> W);

    /**
     * \brief Check status codes and other
     * diagnostic data and issue helpful ROS Messages
     */
    void controllerDiagnosis();

    /**
     * \brief Calculate the control input
     */
    void calculateControlInput();
    /**
     * \brief Shift the state and control acadoVariables
     */
    void shiftStatesAndControl();

private:
    std::vector<float> W_;
};

#endif
