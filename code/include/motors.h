#ifndef MOTOR_H
#define MOTOR_H

#include "robot.h"

void motors_init(void);
void motors_set_cmd_right(RobotHandle robot_handle);
void motors_set_cmd_left(RobotHandle robot_handle);
void motors_get_feedback(RobotHandle robot_handle);

#endif
