#ifndef PID_H
#define PID_H
#include <vector>

class PID {
private:
  std::vector<double> p;
  std::vector<double> dp;
  double _err_tolerance;
  int _steps;
  double _best_err;
  
  double cte_prev_;
  double cte_;
  double cte_mem_;

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

  void Twiddle(double cte);
  
  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
