#ifndef ROBOT_H
#define ROBOT_H

#include "zarduino/module/mpu6050.h"
#include "constants.h"

#define PWM_1 0
#define PWM_2 1
#define PWM_COUNT 2

#define U_1 0
#define U_2 1
#define U_COUNT 2

#define Y_THETA 0
#define Y_THETA_DOT 1
#define Y_PSI_1_DOT 2
#define Y_PSI_2_DOT 3
#define Y_PHI_DOT 4
#define Y_COUNT 5

typedef struct {
    MPU6050Config mpu6050_config;
    MPU6050Data mpu6050_data;
    float seconds, dt;
    uint8_t active;
    float pwm[PWM_COUNT];
    float u[U_COUNT];
    float y[Y_COUNT];
} RobotBase;

inline float get_velocity(float *y)
{
    return R*y[Y_THETA_DOT] + (0.5*R)*y[Y_PSI_1_DOT] + (0.5*R)*y[Y_PSI_2_DOT];
}

typedef void *RobotHandle;

RobotHandle robot_create(void);
void robot_init(RobotHandle robot_handle);
void robot_loop_active(RobotHandle robot_handle);
void robot_loop_inactive(RobotHandle robot_handle);

#endif
