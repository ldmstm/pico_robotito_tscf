#include "fpid.h"
#include <string.h>

void Fpid_init(Fpid *pid)
{
  if (!pid) return;
  memset(pid, 0, sizeof(Fpid));
}

void Fpid_init_with_limits(Fpid *pid, float min_out, float max_out)
{
  if (!pid) return;
  memset(pid, 0, sizeof(Fpid));
  pid->min_out = min_out;
  pid->max_out = max_out;
}

float Fpid_compute(Fpid *pid, float setpoint, float measure, float time_step)
{
  if (!pid) return 0.0f;

  float out;

  float error = setpoint - measure;
  float new_ki_term = pid->ki_term + pid->ki * error * time_step;

  out = pid->kf * setpoint;
  out += pid->kp * error;
  out += new_ki_term;

  if (pid->kd != 0.0f)
  {
    // derivative on measurement
    float kd_term = pid->kd * (measure - pid->prev_measure) / time_step; 
    out -= kd_term;
  }

  if (out > pid->max_out)
  {
    out = pid->max_out;
  }
  else if (out < pid->min_out)
  {
    out = pid->min_out;
  }
  else
  {
    pid->ki_term = new_ki_term; // only here, for windup protection
  }

  pid->prev_measure = measure;

  return out;
}

void Fpid_update_k_independent(Fpid *pid, float kf, float kp, float ki, float kd)
{
  if (!pid) return;
  pid->kf = kf;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

void Fpid_update_k_dependent_ideal(Fpid *pid, float kf, float kc, float Ti, float Td)
{
  if (!pid) return;
  pid->kf = kf;
  pid->kp = kc;
  pid->ki = kc / Ti;
  pid->kd = kc * Td;
}

void Fpid_reset_errors(Fpid *pid, float measure)
{
  if (!pid) return;
  pid->ki_term = 0;
  pid->prev_measure = measure;
}
