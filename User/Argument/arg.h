#ifndef __ARG_H
#define __ARG_H

/**
  * @class Macro
  * @brief Global
  */
#define ON 1
#define OFF 0

/**
  * @class Macro
  * @brief PID parameters
  */
/* Controller parameters */
#define PID_KP  2.0f
#define PID_KI  0.5f
#define PID_KD  0.25f

#define PID_TAU 0.02f

#define PID_LIM_MIN -10.0f
#define PID_LIM_MAX  10.0f

#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT  5.0f

#define SAMPLE_TIME_S 0.01f

/* Maximum run-time of simulation */
#define SIMULATION_TIME_MAX 4.0f

/* Filter Switch */
#define FILTER_OFF 0
#define FILTER_ON 1
#define LOW_PASS_FILTER 1

/**
  * @class Variables
  * @brief Gyro
  */
float pitch, roll,  yaw;
short aacx,  aacy,  aacz;       // original acceleration from sensor
short gyrox, gyroy, gyroz;      // original angular acceleration from sensor        
short temp;                     // temperature

#endif