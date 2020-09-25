#ifndef ROBOT_H
#define ROBOT_H

#include "zarduino/module/mpu6050.h"

typedef struct {
    MPU6050Config mpu6050_config;
    MPU6050Data mpu6050_data;
    float seconds, dt;
    uint8_t active;
    float motor_cmd_left, motor_cmd_right;
    float psi_left_dot, psi_right_dot;
} RobotBase;

typedef void *RobotHandle;

void robot_init(RobotHandle robot_handle);
void robot_loop(RobotHandle robot_handle);

#endif
