#ifndef MOTOR_H
#define MOTOR_H

#include "robot.h"

void motors_set_left(Robot *robot, float duty_cycle, uint8_t dir);
void motors_set_right(Robot *robot, float duty_cycle, uint8_t dir);
void motors_init(Robot *robot);
void motors_get_feedback(Robot *robot, float dt);

#endif
