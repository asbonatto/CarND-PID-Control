#ifndef Twiddler_H
#define Twiddler_H

#include "PID.h"
#include <vector>
#include <numeric>
#include <limits>
#include <iostream>
#include <sstream>

using namespace std;

class Twiddler {
public:
  /*
  * Controllers
  */
  PID pid_steer;
  PID pid_throttle;
  
  double steer_, throttle_;
  
  // Optimizer switch
  bool optimize_;
  
  // Variable to finish the simulation. TODO: should not be public
  bool restart_simulation_;
  
  // Optimizer variables
  double err_, best_err_;
  double step_rate_, dp_;
  vector<double> p_;

  int branch_, nruns_, nsteps_, v_;

  /*
  * Constructor
  */
  Twiddler();

  /*
  * Destructor.
  */
  virtual ~Twiddler();

  /*
  * Initialize PID.
  */
  void Init(bool optimize, double Kp_1, double Ki_1, double Kd_1, double Kp_2, double Ki_2, double Kd_2);

  void UpdateError(double cte, double speed_error);
  void UpdateGradient();
  bool is_saturated(double command);
  void ResetPIDs();
  
};

#endif /* PID_H */
