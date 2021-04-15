#ifndef MARCH_MODEL_PREDICTIVE_CONTROLLER_H
#define MARCH_MODEL_PREDICTIVE_CONTROLLER_H

#include <acado_auxiliary_functions.h>
#include <iostream>
#include <vector>

using namespace std;

class ModelPredictiveController {

public:
    ModelPredictiveController(std::vector<float> W);

    // Public variables
    std::vector<double> command;
    std::string joint_name;
    double cost; // Objective value

    // Timing variables
    acado_timer t;
    double t_preparation, t_feedback;

    // status variables
    int preparationStepStatus;
    int feedbackStepStatus;

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
