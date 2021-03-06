#ifndef PID_H
#define PID_H

#include <iostream>
#include <limits>       // std::numeric_limits
#include <cmath>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);
  void Twiddle(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  double best_error = std::numeric_limits<double>::max();
  int index = 0;
  int state_M = 0;

};

#endif /* PID_H */
