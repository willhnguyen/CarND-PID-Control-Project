#ifndef FUZZY_PID_H
#define FUZZY_PID_H

#include "PID.h"

class FuzzyPID : public PID {
public:
  double Kp_tm1; // Kp(t-1)
  double Kp_tm2; // Kp(t-2)
  double V_tm1;  // V(t-1)
  double t_tm1;  // time(t-1)

  // Magnitudes to use
  double large_mag;
  double medium_mag;
  double small_mag;

  /*
  * Constructor
  */
  FuzzyPID();

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, double speed, double t);

private:
  int _iterations;
};

#endif /* FUZZY_PID_H */
