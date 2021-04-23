#define _USE_MATH_DEFINES

#include "model_predictive_controller.hpp"
#include "acado_common.h"
#include <acado_auxiliary_functions.h>
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

ModelPredictiveController::ModelPredictiveController(
    std::vector<float> W, ModelPredictiveControllerConstraints constraints)
    : W_(W)
    , constraints_(constraints)
{
}

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

    // Assign the weighting matrix
    assignWeightingMatrix(W_);

    // Assign the constraints
    assignConstraints(constraints_);

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

void ModelPredictiveController::assignConstraints(
    ModelPredictiveControllerConstraints constraints)
{
    std::vector<float> lb_effort;
    lb_effort.reserve(constraints.effort.size());
    std::transform(constraints.effort.begin(), constraints.effort.end(),
        lb_effort.begin(), [](float & v) {
            return v * -1.0;
        });

    std::vector<float> lb_velocity;
    lb_velocity.reserve(constraints.velocity.size());
    transform(constraints.velocity.begin(), constraints.velocity.end(),
        lb_velocity.begin(), [](float & v) {
            return v * -1.0;
        });

    std::vector<float> lb_states, ub_states;
    lb_states.reserve(ACADO_NX);
    ub_states.reserve(ACADO_NX);
    for (int i = 0; i < constraints.position_lower.size(); ++i) {
        lb_states.insert(
            lb_states.end(), { constraints.position_lower[i], lb_velocity[i] });
        ub_states.insert(ub_states.end(),
            { constraints.position_upper[i], constraints.velocity[i] });
    }

    for (int i = 0; i < ACADO_N; ++i) {
        std::copy(lb_effort.begin(), lb_effort.end(),
            std::begin(acadoVariables.lbValues) + i * ACADO_NU);

        std::copy(constraints.effort.begin(), constraints.effort.end(),
            std::begin(acadoVariables.ubValues) + i * ACADO_NU);

        std::copy(lb_states.begin(), lb_states.end(),
            std::begin(acadoVariables.lbAValues) + i * ACADO_NX);

        std::copy(ub_states.begin(), ub_states.end(),
            std::begin(acadoVariables.ubAValues) + i * ACADO_NX);
    }
}

void ModelPredictiveController::controllerDiagnosis()
{
    // Check acado_preparationStep() status code
    ROS_WARN_STREAM_COND(preparationStepStatus >= PREP_INTERNAL_ERROR,
        "Error in preparation step");

    // Check acado_feedbackStep() status code
    // Only checks codes that indicate an error
    switch (feedbackStepStatus) {
        case QP_ITERATION_LIMIT_REACHED:
            ROS_WARN(
                "QP could not be solved within the given number of iterations");
            break;

        case QP_INTERNAL_ERROR:
            ROS_WARN("QP could not be solved due to an internal error");
            break;

        case QP_INFEASIBLE:
            ROS_WARN("QP is infeasible and thus could not be solved");
            break;

        case QP_UNBOUNDED:
            ROS_WARN("QP is unbounded and thus could not be solved");
            break;
    }
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
