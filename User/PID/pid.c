#include "PID.h"

void PIDController_Init(PIDController *pid) {

  /* Clear controller variables */
  pid->integrator = 0.0f;
  pid->prevError  = 0.0f;

  pid->differentiator  = 0.0f;
  pid->prevMeasurement = 0.0f;

  pid->out = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

      /*
      * Error signal
      */
  float error = setpoint - measurement;

      /*
      * Proportional
      */
  float proportional = pid->Kp * error;

      /*
      * Integral
      */
  pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

      /* Anti-wind-up via integrator clamping */
  if (pid->integrator > pid->limMaxInt) {

      pid->integrator = pid->limMaxInt;

  } else if (pid->integrator < pid->limMinInt) {

      pid->integrator = pid->limMinInt;

  }

      /*
      * Derivative (band-limited differentiator)
      */
              
  pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement));	/* Note: derivative on measurement, therefore minus sign in front of equation! */
  switch(pid->filter){
  case 0:
    break;
  case 1:       // low-pass filter
    pid->differentiator += (2.0f * pid->tau - pid->T) * pid->differentiator
                          / (2.0f * pid->tau + pid->T);
  }

      /*
      * Compute output and apply limits
      */
  pid->out = proportional + pid->integrator + pid->differentiator;

  if (pid->out > pid->limMax) {
      pid->out = pid->limMax;
  } 
  else if (pid->out < pid->limMin) {
      pid->out = pid->limMin;
  }

      /* Store error and measurement for later use */
  pid->prevError       = error;
  pid->prevMeasurement = measurement;

      /* Return controller output */
  return pid->out;
}

float PIDController_Update_2Stage(PIDController *pid_outer, PIDController *pid_inner,
                                  float setpoint_outer,
                                  float measurement_outer, float measurement_inner){
  float setpoint_inner = PIDController_Update(pid_outer, setpoint_outer, measurement_outer);
  return PIDController_Update(pid_inner, setpoint_inner, measurement_inner);
}

float PIDController_Update_nStage(PIDController** pids, unsigned int stages,
                                  float setpoint_outest,
                                  float* measurements){
  unsigned int i;
  float out, pt = setpoint_outest;
  for(i = 0; i < stages; i++){
    out = PIDController_Update(pids[i], pt, measurements[i]);
    pt = out;
  }
  return out;
}
