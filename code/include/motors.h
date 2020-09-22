#ifndef MOTOR_H
#define MOTOR_H

#include "robot.h"

void motors_init(Robot *robot);
void motors_set_left(Robot *robot, float signed_duty_cycle);
void motors_set_right(Robot *robot, float signed_duty_cycle);
void motors_get_feedback(Robot *robot, float dt);

#endif
