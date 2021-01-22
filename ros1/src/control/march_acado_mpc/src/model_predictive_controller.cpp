#define _USE_MATH_DEFINES

#include "model_predictive_controller.hpp"
#include "acado_common.h"

#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

// Global variables used by the solver
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

void ModelPredictiveController::init() {
  // Reset all solver memory
  memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
  memset(&acadoVariables, 0, sizeof(acadoVariables));

  // Initialize the solver
  acado_initializeSolver();

  // Prepare a consistent initial guess
  for (int i = 0; i < ACADO_N + 1; ++i) {
    acadoVariables.x[i * ACADO_NX] = 0; // theta
    acadoVariables.x[i * ACADO_NX + 1] = 0; // dtheta
  }

  double theta_ref = 90*(M_PI/180);

  // Prepare references (step reference)
  for (int i = 0; i < ACADO_N; ++i) {
    acadoVariables.y[i * ACADO_NY + 0] = theta_ref; // theta
    acadoVariables.y[i * ACADO_NY + 1] = 0;         // dtheta
    acadoVariables.y[i * ACADO_NY + 2] = 0;         // T
  }

  acadoVariables.yN[0] = theta_ref; // theta
  acadoVariables.yN[1] = 0;         // dtheta
  acadoVariables.yN[2] = 0;         // T

  // Current state feedback
  setInitialState(x0);

  // Warm-up the solver
  acado_preparationStep();

}

void ModelPredictiveController::setInitialState(vector<double> x0) {
  for (int i = 0; i < ACADO_NX; ++i) {
    acadoVariables.x0[i] = x0[i];
  }
}

void ModelPredictiveController::calculateControlInput() {

  // Set initial speed
  setInitialState(x0);

  // preparation step
  acado_preparationStep();

  // feedback step
  acado_feedbackStep( );
  u = acadoVariables.u[0];

  // Shift states and control and prepare for the next iteration
  acado_shiftStates(2, 0, 0);
  acado_shiftControls( 0 );

}

