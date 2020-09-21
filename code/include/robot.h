#ifndef ROBOT_H
#define ROBOT_H

#include "zarduino/core/pins.h"
#include "zarduino/module/oled.h"
#include "zarduino/module/mpu6050.h"
#include "zarduino/module/radio.h"

#include "control.h"

typedef enum {
    ROBOT_STATE_PRE_CONTROL,
    ROBOT_STATE_CONTROL,
    ROBOT_STATE_PRE_PASSIVE,
    ROBOT_STATE_PASSIVE,
    ROBOT_STATE_PRE_MOTOR,
    ROBOT_STATE_MOTOR,
    ROBOT_STATE_COUNT
} RobotState;

typedef struct {
    Pin motor_left_pwm;
    Pin motor_left_dir;
    Pin motor_left_feedback;
    Pin motor_right_pwm;
    Pin motor_right_dir;
    Pin motor_right_feedback;

    Pin adc_pin;
    Pin button_1_pin;
    Pin button_2_pin;
    Pin button_3_pin;
    Pin led_pin;

    Pin radio_ce_pin;
    Pin radio_csn_pin;

    float motor_right_vel;
    float motor_left_vel;

    OLEDConfig oled_config;

    MPU6050Config mpu6050_config;
    MPU6050Data mpu6050_data;

    RadioConfig radio_config; 

    ControlState control_state;
    RobotState state;
    float seconds;
} Robot;

Robot robot_create(void);
void robot_init(Robot *robot);
void robot_loop(Robot *robot);

#endif
