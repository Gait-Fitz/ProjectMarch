#ifndef MARCH_MODEL_PREDICTIVE_CONTROLLER_H
#define MARCH_MODEL_PREDICTIVE_CONTROLLER_H

#include <acado_auxiliary_functions.h>
#include <iostream>
#include <vector>

using namespace std;

struct ModelPredictiveControllerConstraints {
    std::vector<float> effort;
    std::vector<float> position_lower;
    std::vector<float> position_upper;
    std::vector<float> velocity;
};

class ModelPredictiveController {

public:
    ModelPredictiveController(std::vector<float> W);

    /**
     * Controller variables
     */

    std::vector<double> command; // calculated input

    /**
     * Diagnostic variables
     */

    // Timing variables
    acado_timer t;
    double t_preparation, t_feedback;

    // Status variables
    int preparationStepStatus;
    int feedbackStepStatus;

    // Performance variables
    double cost; // Objective value

    // qpOASES error enums
    enum Error {

        // acado_preparationStep() errors
        PREP_INTERNAL_ERROR = 1,

        // acado_feedbackStep() errors
        QP_ITERATION_LIMIT_REACHED = 1,
        QP_INTERNAL_ERROR = -1,
        QP_INFEASIBLE = -2,
        QP_UNBOUNDED = -3
    };

    /**
     * \brief Initialise the model predictive controller
     */
    void init();

    /**
     * \brief Set the initial state
     * @param x0 - initial state
     */
    void setInitialState(int joint, vector<double> x0);

    /**
     * \brief Set the reference for time step n in [0, N]
     * @param n
     * @param reference
     */
    void setReference(int joint, int n, const std::vector<double>& states,
        const std::vector<double>& inputs);

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
    std::vector<double> calculateControlInput();
    /**
     * \brief Shift the state and control acadoVariables
     */
    void shiftStatesAndControl();

private:
    std::vector<float> W_;
};

#endif
