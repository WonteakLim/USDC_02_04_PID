#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error_;
  double i_error_;
  double d_error_;

  /*
  * Coefficients
  */ 

  double Kp_;
  double Ki_;
  double Kd_;

  /*
  * Ctrl Cmd
  */
  double cmd_;

  /*
  * Twiddle
  */
  bool is_twiddle_;
  int window_size_;
  int window_step_;
  int update_param_index_;
  int update_subiter_;
  double total_error_;

  int num_param_;
  std::vector<double> p;
  std::vector<double> dp;

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

  /*
  * Get Control input
  */
  inline double GetCtrlInput() { return cmd_; }
};

#endif /* PID_H */
