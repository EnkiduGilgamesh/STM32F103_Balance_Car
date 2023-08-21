#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {

  /* Controller gains */
  float Kp;
  float Ki;
  float Kd;

  /* Derivative low-pass filter time constant */
  float tau;

  /* Output limits */
  float limMin;
  float limMax;
  
  /* Integrator limits */
  float limMinInt;
  float limMaxInt;

  /* Sample time (in seconds) */
  float T;

  /* Controller "memory" */
  float integrator;
  float prevError;			/* Required for integrator */
  float differentiator;
  float prevMeasurement;		/* Required for differentiator */

  /* Controller output */
  float out;
  
  /* Filter Switch */
  unsigned int filter;

} PIDController;

void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);

/* Series-wound Stage 2 */
float PIDController_Update_2Stage(PIDController *pid_outer, PIDController *pid_inner,
                                  float setpoint_outer,
                                  float measurement_outer, float measurement_inner);

/* Series-wound Stage n */
float PIDController_Update_nStage(PIDController** pids, unsigned int stages,    /* Sort your pids and measurements from the outest to innest */
                                  float setpoint_outest,
                                  float* measurements);

#endif
