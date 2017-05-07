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

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  //previous cte
  double prev_cte;
  //integrated cte
  double int_cte;
  //steering
  double steering;
  //flag for initialization of int_cte and prev_cte
  bool isFirst;
  //total error
  double error;
  //count for error mean
  long error_count;
  //total count
  long count;

  //tiggle optimization flag
  bool twiggle;
  //best gained error
  double best_err = 9999999.0;
  //parameter to optimize
  double p[3] = {0.08,0.0002,0.8};
  //best parameter optimized
  double best_p[3] = {0.08,0.0002,0.8};
  //delta parameter
  double dp[3] = {0.002, 0.00002, 0.02};
  //current parameterto be optimised
  int parameterIndex = 0;
  //direction for parameter adaptation
  bool twiggleUp = true;
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
  void Init(double Kp, double Ki, double Kd, bool twiggle);

  /*
  * Update the PID error variables given cross track error and calculate the steering.
  */
  void UpdateError(double cte);

  /*
  * Answer the total error.
  */
  double GetTotalError();

  /*
  * Answer the steering
  */
  double GetSteering();

  /** tune parameter recursive */
  void TwiggleParameter(double error, int paramIndex);

  /** answer the best parameter found*/
  void GetBestTwiggleParam(int paramIndex);


};

#endif /* PID_H */
