#ifndef FUZZY_PID_H
#define FUZZY_PID_H

#include "PID.h"

class FuzzyPID : public PID {
public:
  double V_prev;
  double delta_V_prev;
  double dVdKp_prev;
  double time_prev;

  /*
  * Constructor
  */
  FuzzyPID();

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, double speed, double t);
};

#endif /* FUZZY_PID_H */
