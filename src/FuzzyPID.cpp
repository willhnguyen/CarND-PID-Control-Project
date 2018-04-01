#include "FuzzyPID.h"

using namespace std;

#define EPSILON 1E-10

FuzzyPID::FuzzyPID() {
    // Fill the error and coefficient terms with default value zero
    Init(0, 0, 0);

    // Initialize Fuzzy PID's state variables
    V_prev = 0;
    delta_V_prev = 0;
    dVdKp_prev = 0;
    time_prev = 0;
}

void FuzzyPID::UpdateError(double error, double speed, double t) {
    // Error can be...
    // * CTE for steering angle
    // * (actual - desired) for other measurements

    double error_previous = p_error;
    double delta_t = t - time_prev;

    // Update Kp's value based on previous error calculations
    double delta_Kp = 1E-10;
    if (delta_V_prev > EPSILON) {
      delta_Kp = 1E-1;

    } else if (delta_V_prev < -EPSILON) {
      delta_Kp = 1E-5;
    }

    // Determine the sign of delta_Kp and add to Kp
    // Signs inverse of the signs suggested in the paper work best
    if (dVdKp_prev >= 0) {
      Kp += delta_Kp;
    } else {
      Kp -= delta_Kp;
    }

    // EQUATION: p_error(t) = error(t);
    p_error = error;

    // EQUATION: i_error(t) = error(t) + error(t-1) + ... + error(0)
    i_error += error;

    // EQUATION: d_error(t) = (error(t) - error(t-1)) / delta_t
    // Assumption: delta_t is consistently 1
    d_error = (error - error_previous) / delta_t;


    // // Fuzzy Kp Update Logic
    // V Calculation
    // EQUATION: V(e,e_dot) = 1.0 / 2.0 * (e**2 + e_dot**2)
    double V = 0.5 * (error * error + d_error * d_error);
    // float V = 0.5 * (error * error + speed * speed);
    double delta_V = V - V_prev;
    double dVdKp = delta_V / delta_Kp;

    // Store new state
    V_prev = V;
    delta_V_prev = delta_V;
    dVdKp_prev = dVdKp;
    time_prev = t;
}
