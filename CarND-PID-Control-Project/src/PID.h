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
  * Coefficients Speed
  */ 
  double Kp;
  double Ki;
  double Kd;
  /*
  * Coefficients throttle
  */
  double Kp_t;
  double Ki_t;
  double Kd_t;


  //previous cte
  double prev_cte;
  //integrated cte
  double int_cte;
  //steering
  double steering;
  //throttle
  double throttle;
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
  double p[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
  //best parameter optimized
  double best_p[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
  //delta parameter
  double dp[6] = {0.002, 0.00002, 0.02, 0.05, 0.00005, 0.005};
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
  void Init(double aKp, double aKi, double aKd, double aKp_t, double aKi_t, double aKd_t, bool doTwiggle);

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

  /*
  * Answer the steering
  */
  double GetThrottle();

  /** tune parameter recursive */
  void TwiggleParameter(double error, int paramIndex);

  /** answer the best parameter found*/
  void GetBestTwiggleParam(int paramIndex);


};

#endif /* PID_H */
