#ifndef MARCH_MODEL_PREDICTIVE_CONTROLLER_H
#define MARCH_MODEL_PREDICTIVE_CONTROLLER_H

#include <acado_auxiliary_functions.h>
#include <iostream>
#include <vector>

using namespace std;

class ModelPredictiveController {

public:
    // Public variables
    vector<double> x0 { 0, 0 }; // Current state
    double u; // Calculated control input
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
};

#endif
