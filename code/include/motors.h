#ifndef MOTOR_H
#define MOTOR_H

#include "robot.h"

void motors_init(void);
void motors_set_pwm_1(float *signed_pwm);
void motors_set_pwm_2(float *signed_pwm);
void motors_get_feedback(float *psi_1_dot, float *psi_2_dot, float dt);

#endif
