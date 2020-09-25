#ifndef ROBOT_H
#define ROBOT_H

#include "zarduino/module/mpu6050.h"

typedef struct {
    MPU6050Config mpu6050_config;
    MPU6050Data mpu6050_data;
    float seconds, dt;
    uint8_t active;
    float theta, theta_dot, phi_dot;
    float psi_left_dot, psi_right_dot;
    float motor_cmd_left, motor_cmd_right;
} RobotBase;

typedef void *RobotHandle;

RobotHandle robot_create(void);
void robot_init(RobotHandle robot_handle);
void robot_loop(RobotHandle robot_handle);

#endif
