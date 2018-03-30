#include "PID.h"

using namespace std;

/*
* Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

const double tolerance = 0.01;

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::Twiddle(double cte) {
  double p[] = {Kp, Ki, Kd};
  double dp[] = {p_error, i_error, d_error};

  if ((p_error + i_error + d_error) > tolerance) {
    switch (state_M) {
      case 0: {
        p[index] += dp[index];
        state_M = 1;
        if (fabs(cte) < fabs(best_error)) {
          best_error = cte;
          dp[index] *= 1.1;
          index = (index + 1) % 3;
          state_M = 0;
        }
        else {
          p[index] -= 2 * dp[index];
          state_M = 1;
        }
        break;
      }
      case 1: {
        if (fabs(cte) < fabs(best_error)) {
          best_error = cte;
          dp[index] *= 1.1;
        }
        else {
          p[index] += dp[index];
          dp[index] *= 0.9;
        }
        index = (index + 1) % 3;
        state_M = 0;
        break;
      }
    }
    p_error = dp[0];
    i_error = dp[1];
    d_error = dp[2];
    Kp = p[0];
    Ki = p[1];
    Kd = p[2];
  }
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
	i_error += cte;
	p_error = cte;
}

double PID::TotalError() {
  // -tau_p * CTE - tau_d * diff_CTE - tau_i * int_CTE
	return -(Kp * p_error + Ki * i_error + Kd * d_error);
}
