#define _USE_MATH_DEFINES

#include "model_predictive_controller.hpp"
#include "acado_auxiliary_functions.h"
#include "acado_common.h"
#include "acado_qpoases_interface.hpp"
#include <ros/console.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

// Global variables used by the solver
ACADOvariables acadoVariables = {};
ACADOworkspace acadoWorkspace = {};

void ModelPredictiveController::init()
{
    // Initialize the solver
    acado_initializeSolver();

    // Initialize state array with zero
    std::fill(std::begin(acadoVariables.x), std::end(acadoVariables.x), 0.0);

    // Initialize input array with zero
    std::fill(std::begin(acadoVariables.u), std::end(acadoVariables.u), 0.0);

    // Initialize "running" and "end" reference array with zero
    std::fill(std::begin(acadoVariables.y), std::end(acadoVariables.y), 0.0);
    std::fill(std::begin(acadoVariables.yN), std::end(acadoVariables.yN), 0.0);

    // Initialize "running" and "end" weighting array with zero
    std::fill(std::begin(acadoVariables.W), std::end(acadoVariables.W), 0.0);
    std::fill(std::begin(acadoVariables.WN), std::end(acadoVariables.WN), 0.0);

    // Reserve space for the inputs
    command.reserve(ACADO_NU);

    // Warm-up the solver
    acado_preparationStep();
}

void ModelPredictiveController::setInitialState(int joint, vector<double> x0)
{
    std::copy(x0.begin(), x0.end(),
        std::begin(acadoVariables.x0) + joint * x0.size());
}

void ModelPredictiveController::setReference(int joint, int n,
    const std::vector<double>& states, const std::vector<double>& inputs)
{

    // check if size of reference at time step n is equal to size of ACADO_NY
    if (ACADO_NY != states.size() + inputs.size()) {
        ROS_DEBUG_STREAM_ONCE(
            "The supplied reference vector has an incorrect size");
    }

    // Set the reference of node n
    if (n != ACADO_N) {
        // set "running" reference
        // reference of node 0 to node ACADO_N-1, includes states and control
        // references
        std::copy(states.begin(), states.end(),
            std::begin(acadoVariables.y) + joint * states.size()
                + n * ACADO_NY);
        std::copy(inputs.begin(), inputs.end(),
            std::begin(acadoVariables.y) + ACADO_NX + joint + n * ACADO_NY);
    } else {
        // set "end" reference
        // reference of the last node N, includes only the states references
        std::copy(states.begin(), states.end(),
            std::begin(acadoVariables.y) + joint * states.size()
                + n * ACADO_NY);
    }
}

void ModelPredictiveController::shiftStatesAndControl()
{
    acado_shiftStates(/*strategy=*/2, /*xEnd=*/0, /*uEnd=*/0);
    acado_shiftControls(/*uEnd=*/0);
}

void ModelPredictiveController::assignWeightingMatrix(std::vector<float> W)
{
    // set the diagonal of the ACADO W matrix (state and input weights)
    for (int i = 0; i < ACADO_NY; ++i) {
        acadoVariables.W[i * (ACADO_NY + 1)] = W[i];
    }

    // Set the diagonal of the ACADO WN matrix (only state weights)
    for (int i = 0; i < ACADO_NYN; ++i) {
        acadoVariables.WN[i * (ACADO_NYN + 1)] = W[i];
    }
}

void ModelPredictiveController::controllerDiagnosis()
{
    // Check acado_preparationStep() status code
    ROS_WARN_STREAM_COND(preparationStepStatus != 0,
        acado_getErrorString(preparationStepStatus));

    // Check acado_feedbackStep() status code
    ROS_WARN_STREAM_COND(
        feedbackStepStatus != 0, acado_getErrorString(feedbackStepStatus));
}

std::vector<double> ModelPredictiveController::calculateControlInput()
{
    // Preparation step (timed)
    acado_tic(&t);
    preparationStepStatus = acado_preparationStep();
    t_preparation = acado_toc(&t);

    // Feedback step (timed)
    acado_tic(&t);
    feedbackStepStatus = acado_feedbackStep();
    t_feedback = acado_toc(&t);

    // Objective cost for diagnosis
    cost = acado_getObjective();

    // Perform a diagnosis on the controller
    controllerDiagnosis();

    // get command
    command.insert(command.begin(), std::begin(acadoVariables.u),
        std::begin(acadoVariables.u) + ACADO_NU);

    // return command
    return command;
}
