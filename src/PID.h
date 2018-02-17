#ifndef PID_H
#define PID_H

#include <math.h>
#include <string>
#include <iostream>
using namespace std;


class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double p_cum, i_cum = 0, d_cum = 0;

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
  void Init(double Kp, double Ki, double Kd, string name);

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
