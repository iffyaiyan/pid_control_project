#ifndef PID_H
#define PID_H
#include <iostream>
#include <math.h>

enum PID_Type { Steer, Speed, Throttle};

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double prev_cte;
  double cte_i;
  double cte;

  /* Twiddle*/
  PID_Type pid_type;
  double p[3];
  double dp[3];
  double best_err;
  //bool second_step;
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
  void Init(double Kp_, double Ki_, double Kd_,PID_Type pid_type);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte_);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*Estimate parameters*/
  void Tunning(double error,int index,bool second_step);
};

#endif /* PID_H */
