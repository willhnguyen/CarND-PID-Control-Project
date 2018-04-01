#include "PID.h"

using namespace std;

PID::PID() {
    // Fill the error and coefficient terms with default value zero
    Init(0, 0, 0);
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    // Set coefficient terms with the ones passed in
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    // Reset error terms to zero
    p_error = 0;
    i_error = 0;
    d_error = 0;
}

void PID::UpdateError(double error) {
    // Error can be...
    // * CTE for steering angle
    // * (actual - desired) for other measurements

    double error_previous = p_error;

    // EQUATION: p_error(t) = error(t);
    p_error = error;

    // EQUATION: i_error(t) = error(t) + error(t-1) + ... + error(0)
    i_error += error;

    // EQUATION: d_error(t) = (error(t) - error(t-1)) / delta_t
    // Assumption: delta_t is consistently 1
    d_error = error - error_previous;
}

double PID::TotalError() {
    return -Kp * p_error -Ki * i_error -Kd * d_error;
}
