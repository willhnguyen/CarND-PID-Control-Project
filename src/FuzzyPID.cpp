#include "FuzzyPID.h"
#include <iostream>
#include <cmath>

using namespace std;

#define EPSILON 1E-10

FuzzyPID::FuzzyPID() {
  // Fill the error and coefficient terms with default value zero
  Init(0, 0, 0);
  _iterations = 0;

  // Set Fuzzy PID's delta_Kp magnitude values
  large_mag = 0.001;
  medium_mag = 0.0005;
  small_mag = 0.0001;

  // Initialize values necessary to be initialized
  u_tm2 = 1;
  u_tm1 = 1;
}

void FuzzyPID::UpdateError(double error, double speed, double t) {

    /***************************************************************************
    * Adaptive Kp Calculations
    ***************************************************************************/
    double delta_t = t - t_tm1;
    t_tm1 = t;

    // Calculate V
    double error_previous = p_error;
    double delta_error = error - error_previous;
    de_tm1 = delta_error;
    double e_dot = delta_error / delta_t;
    double V = 0.5 * pow(error, 2) + pow(e_dot, 2);

    // Calculate delta_V and V_dot
    double delta_V = V - V_tm1;
    // double V_dot = delta_V / delta_t; // Unused

    // Calculate delta_Kp
    double delta_Kp = Kp - Kp_tm1;
    // double Kp_dot = delta_Kp / delta_t; // Unused

    // Determine Sign for Kp
    // Although the sign is determined by opposite the sign of
    // (delta_V / delta_Kp), we can use logic to determine this since only the
    // sign is desired, not the actual value.
    bool delta_V_is_neg = delta_V < 0;
    bool delta_Kp_is_neg = delta_Kp < 0;
    bool sign_new_delta_Kp_is_neg = (delta_V_is_neg == delta_Kp_is_neg);

    // Update the stored state values for the next iteration
    Kp_tm2 = Kp_tm1;
    Kp_tm1 = Kp;
    V_tm1 = V;

    // Update Kp after one iteration
    if (_iterations > 2) {

      // Determine the new delta_Kp magnitude
      double delta_Kp_for_updating_Kp = small_mag; // If it's zero
      if (delta_V > EPSILON) {
        // If delta_V is positive, then use large magnitude
        delta_Kp_for_updating_Kp = large_mag;
      } else if (delta_V < -EPSILON) {
        // If delta_V is negative, then use medium magnitude
        delta_Kp_for_updating_Kp = medium_mag;
      }

      // Determine the new delta_Kp sign
      if (sign_new_delta_Kp_is_neg) {
        delta_Kp_for_updating_Kp *= -1;
      } else {
        // delta_Kp_for_updating_Kp *= -1;
      }

      // Update Kp value
      Kp += delta_Kp_for_updating_Kp;
    } else {
      _iterations += 1;
    }

    /***************************************************************************
    * Adaptive Ki Calculations
    ***************************************************************************/
   if (_iterations > 2) {
    // Ki = -epsilon * (dy/du)^-1
    // (dy/du)^-1 is inverse of derivative
    // (dy/du) = cte in this situation
    double Ki_epsilon = 0.00001; // Arbitrary value for now
    double dy = de_tm1;
    if (abs(dy) < EPSILON) {
      // to prevent divide by zero problems
      dy = (dy >= 0 ? 1 : -1) * EPSILON;
    }
    double du = u_tm1 - u_tm2;
    if (abs(du) < EPSILON) {
      // to prevent divide by zero problems
      du = (du >= 0 ? 1 : -1) * EPSILON;
    }
    double dydu = dy / du;
    if (abs(du) < EPSILON) {
      // to prevent divde by zero problems
      dydu = (dydu >= 0 ? 1 : -1) * EPSILON;
    }
    double dydu_inverse = 1 / dydu;

    Ki = - Ki_epsilon * dydu_inverse;
    
    // For debugging
    cout << dy << ", " << du << ", " << dydu << ", " << Ki << endl;
   }

   /***************************************************************************
    * Main Kp, Kd, Ki Calculations
    ***************************************************************************/
    // PID Controller
    // P EQUATION: p_error(t) = error(t);
    // I EQUATION: i_error(t) = error(t) + error(t-1) + ... + error(0)
    // D EQUATION: d_error(t) = (error(t) - error(t-1)) / delta_t
    p_error = error;
    i_error += error;
    // d_error = (error - error_previous) / delta_t;
    d_error = error - error_previous;
}

double FuzzyPID::TotalError() {
  // Calculate the control value and store it for later
  u_tm2 = u_tm1;
  u_tm1 = -Kp * p_error -Ki * i_error -Kd * d_error;
  
  return u_tm1;
}