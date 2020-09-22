#ifndef ROBOT_H
#define ROBOT_H

#include "zarduino/core/pins.h"
#include "zarduino/module/oled.h"
#include "zarduino/module/mpu6050.h"
#include "zarduino/module/radio.h"

#include "control.h"
#include "buffer.h"

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

    OLEDConfig oled_config;

    MPU6050Config mpu6050_config;
    MPU6050Data mpu6050_data;

    RadioConfig radio_config;

    State state;
    ControllerHandle controller_handle;
    float dt;
    float seconds;
    uint8_t active;
} Robot;

void robot_set_config(Robot *robot);
void robot_init(Robot *robot);
void robot_loop(Robot *robot);

#endif
