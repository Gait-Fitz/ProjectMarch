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

    // Current state feedback
    setInitialState(x0);

    // Warm-up the solver
    acado_preparationStep();
}

void ModelPredictiveController::setInitialState(vector<double> x0)
{
    for (int i = 0; i < ACADO_NX; ++i) {
        acadoVariables.x0[i] = x0[i];
    }
}
void ModelPredictiveController::setReference(
    int n, const std::vector<double>& reference)
{

    // check if size of reference at time step n is equal to size of ACADO_NY
    if (ACADO_NY != reference.size()) {
        ROS_DEBUG_STREAM_ONCE(joint_name
            << ", The supplied reference vector has an incorrect size");
    }

    // Set the reference of node n
    if (n != ACADO_N) {
        // set "running" reference
        // reference of node 0 to node ACADO_N-1, includes states and control
        // references
        std::copy_n(reference.begin(), ACADO_NY,
            std::begin(acadoVariables.y) + n * ACADO_NY);
    } else {
        // set "end" reference
        // reference of the last node N, includes only the states references
        std::copy_n(
            reference.begin(), ACADO_NYN, std::begin(acadoVariables.yN));
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

void ModelPredictiveController::calculateControlInput()
{
    // Set initial state
    setInitialState(x0);

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

    // Set mpc command
    u = acadoVariables.u[0];

    // Shift states and control and prepare for the next iteration
    acado_shiftStates(/*strategy=*/2, /*xEnd=*/0, /*uEnd=*/0);
    acado_shiftControls(/*uEnd=*/0);

    // Perform a diagnosis on the controller
    controllerDiagnosis();
}
