#ifndef PID_H
#define PID_H
#include <vector>

class PID {
private:
  bool twiddle_;
  bool tried_adding_;
  bool tried_subtracting_;
  
  std::vector<double> p;
  std::vector<double> dp;
  int step_;
  int number_of_settle_steps_;
  int number_of_steps_to_twiddle_;
  short actual_param_to_twiddle_;
  double error_tolerance_;
  double best_error_;
  double total_error_;
  
  double cte_prev_;
  double cte_;
  double cte_mem_;
  
  void Twiddle(double cte);
  void NextTwiddleParameter();

public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  
  double err;

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
  
  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
