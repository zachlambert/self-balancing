#ifndef ROBOT_H
#define ROBOT_H

#include "zarduino/module/oled.h"
#include "zarduino/module/mpu6050.h"
#include "zarduino/module/radio.h"

#include "state.h"
#include "control.h"

typedef struct {
    OLEDConfig oled_config;
    MPU6050Config mpu6050_config;
    MPU6050Data mpu6050_data;
    RadioConfig radio_config;
    State state;
    ControllerHandle controller_handle;
    uint8_t active;
} Robot;

void robot_init(Robot *robot);
void robot_loop(Robot *robot);

#endif
