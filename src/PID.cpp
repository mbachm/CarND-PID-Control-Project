#include "PID.h"
#include <math.h>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  
  p = {Kp, Ki, Kd};
  dp = {1.0, 1.0, 1.0};
  
  _steps = 0;
  _err_tolerance = 0.2;
  _best_err = 0;
  
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  
  cte_prev_ = cte_;
  cte_ = cte;
  cte_mem_ = cte_mem_ + cte_;
  
  p_error = Kp * cte_;
  i_error = Ki * cte_mem_;
  d_error = Kd * (cte_-cte_prev_);
}

void PID::Twiddle(double cte) {
  if(_steps == 0){
    _best_err = cte;
    p[0] = dp[0];
    p[1] = dp[1];
    p[2] = dp[2];
    ++_steps;
    return;
  }
  
  if(_best_err < _err_tolerance){
    _best_err = cte;
    dp[0] *= 1.1;
    dp[1] *= 1.1;
    dp[2] *= 1.1;
    p[0] += dp[0];
    p[1] += dp[1];
    p[2] += dp[2];
    Kp = p[0];
    Kd = p[1];
    Ki = p[2];
    cout << "better error" << endl;
  } else {
    if (_steps == 1) {
      p[0] -= 2 * dp[0];
      p[1] -= 2 * dp[1];
      p[2] -= 2 * dp[2];
      Kp = p[0];
      Kd = p[1];
      Ki = p[2];
      cout << "worse error, first try" << endl;
      ++_steps;
    } else {
      p[0] += dp[0];
      p[1] += dp[1];
      p[2] += dp[2];
      dp[0] *= 0.9;
      dp[1] *= 0.9;
      dp[2] *= 0.9;
      Kp = p[0];
      Kd = p[1];
      Ki = p[2];
      cout << "worse error, second try" << endl;
      _steps = 1;
    }
    
  }
}

double PID::TotalError() {
}

