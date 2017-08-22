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
  dp = {0.5, 0.5, 0.5};
  
  step_ = 0;
  number_of_settle_steps_ = 200;
  number_of_steps_to_twiddle_ = 2000;
  actual_param_to_twiddle_ = 0;
  error_tolerance_ = 0.2;
  best_error_ = numeric_limits<double>::max();
  twiddle_ = true;
  
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
  
  if (twiddle_
      && step_ > number_of_settle_steps_
      && step_ < (number_of_settle_steps_+ number_of_steps_to_twiddle_)
      && error_tolerance_ > total_error_) {
    Twiddle(cte);
  }
  ++step_;
}

void PID::Twiddle(double cte) {
  double actual_error = pow(cte, 2);
  cout << "step: " << step_ << endl;
  cout << "total error: " << total_error_ << endl;
  cout << "best error: " << best_error_ << endl;
  cout << "actual error: " << actual_error << endl;
  
  if(actual_error < best_error_) {
    best_error_ = actual_error;
    dp[actual_param_to_twiddle_] *= 1.1;
    // next parameter
    NextTwiddleParameter();
  }
  if (!tried_adding_ && !tried_subtracting_){
    p[actual_param_to_twiddle_] += dp[actual_param_to_twiddle_];
    Kp = p[0];
    Ki = p[1];
    Kd = p[2];
    tried_adding_ = true;
  } else if (tried_adding_ && !tried_subtracting_) {
    p[actual_param_to_twiddle_] -= 2 * dp[actual_param_to_twiddle_];
    Kp = p[0];
    Ki = p[1];
    Kd = p[2];
    tried_subtracting_ = true;
  } else {
    p[actual_param_to_twiddle_] += dp[actual_param_to_twiddle_];
    Kp = p[0];
    Ki = p[1];
    Kd = p[2];
    dp[actual_param_to_twiddle_] *= 0.9;
    
    // next parameter
    NextTwiddleParameter();
  }
}

double PID::TotalError() {
  total_error_= p_error + d_error + i_error;
  return total_error_;
}

void PID::NextTwiddleParameter() {
  actual_param_to_twiddle_ = (actual_param_to_twiddle_ + 1) % 3;
  tried_adding_ = false;
  tried_subtracting_ = false;
}
