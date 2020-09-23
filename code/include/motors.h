#ifndef MOTOR_H
#define MOTOR_H

#include "control.h"

void motors_init(void);
void motors_set_cmd_right(State *state);
void motors_set_cmd_left(State *state);
void motors_get_feedback(State *state);

#endif
