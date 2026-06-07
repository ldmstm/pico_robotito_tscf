#ifndef __FPID_H
#define __FPID_H

typedef struct
{
  float min_out;
  float max_out;

  float kf; // forward controller
  float kp;
  float ki;
  float kd;

  float ki_term;
  float prev_measure;
} Fpid;

void Fpid_init(Fpid *pid);
void Fpid_init_with_limits(Fpid *pid, float min_out, float max_out);

float Fpid_compute(Fpid *pid, float setpoint, float measure, float time_step);

void Fpid_update_k_independent(Fpid *pid, float kf, float kp, float ki, float kd);
void Fpid_update_k_dependent_ideal(Fpid *pid, float kf, float kc, float Ti, float Td);

void Fpid_reset_errors(Fpid *pid, float measure);

#endif // __FPID_H
