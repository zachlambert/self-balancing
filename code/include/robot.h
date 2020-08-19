#ifndef ROBOT_H
#define ROBOT_H

#include "zarduino/core/pins.h"
#include "zarduino/module/oled.h"
#include "zarduino/module/mpu6050.h"

typedef struct {
    Pin motor_left_pwm;
    Pin motor_left_dir;
    Pin motor_left_feedback;
    Pin motor_right_pwm;
    Pin motor_right_dir;
    Pin motor_right_feedback;

    Pin button_1_pin;
    Pin button_2_pin;
    Pin led_red_pin;

    float motor_right_vel;
    float motor_left_vel;

    OLEDConfig oled_config;
    OLEDData oled_data;

    MPU6050Config mpu6050_config;
    MPU6050Data mpu6050_data;
} Robot;

Robot robot_create(void);
void robot_init(Robot *robot);
void robot_loop(Robot *robot);

#endif
