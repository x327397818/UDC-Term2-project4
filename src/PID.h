#ifndef PID_H
#define PID_H

#define BUFF_STEP 500
#define EVAL_STEP 3000
#define TOLERANCE 1e-6

#include <cmath>
#include <vector>
#include <limits>

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
  
  // Previous CTE
  double prev_cte;
  
  // Twiddle variables
  std::vector<double> dp, p;
  int step, index, pre_index;
  double best_err, cur_err;
  bool processed;
  
  
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
  
  //Twiddle function
  void Twiddle();
  
};

#endif /* PID_H */
