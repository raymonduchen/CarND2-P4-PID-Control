#ifndef PID_H
#define PID_H

#include <iostream>
#include <cmath>
#include <vector>
using namespace std;

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

  double prev_cte;

  double it;

  double Speed;

  vector<double> p;
  vector<double> dp;

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
  void Init(double Kp, double Ki, double Kd, double Speed);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError(double Kp, double Ki, double Kd);
};

#endif /* PID_H */
