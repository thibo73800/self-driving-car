#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  // Integral over CTE
  double int_cte;
  double prev_cte;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  double sideImax;
  bool infz;
  bool first;
  bool increase;
  double sideSmax;
  double throttle;

  bool pos;

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


  double getNextSteeringAngle(double cte, double speed);
};

#endif /* PID_H */
