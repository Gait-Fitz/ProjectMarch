#ifndef MARCH_MODEL_PREDICTIVE_CONTROLLER_H
#define MARCH_MODEL_PREDICTIVE_CONTROLLER_H

#include <vector>
#include <eigen3/Eigen/Dense>

using namespace std;

class ModelPredictiveController {

//private:
//    Eigen::MatrixXd m(5,5);
////    Eigen::MatrixXd B(5,1);
////    Eigen::MatrixXd C(1,5);
////    Eigen::MatrixXd D(1,1);
////    Eigen::MatrixXd K(5,1);
public:
    // Public variables
    vector<double> x0{0,0}; // Current state
    double u;               // Calculated control input


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
     * \brief Calculate the control input
     * @param x0 - initial state
     * @return u - control input
     */
    void calculateControlInput();

    void estimateState();

};

#endif
