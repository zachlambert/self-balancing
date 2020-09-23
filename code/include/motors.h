#ifndef MOTOR_H
#define MOTOR_H

#include "robot.h"

void motors_init(Robot *robot);
void motors_set_cmd_right(Robot *robot);
void motors_set_cmd_left(Robot *robot);
void motors_get_feedback(Robot *robot, float dt);

#endif
