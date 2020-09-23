#ifndef ROBOT_H
#define ROBOT_H

#include "zarduino/core/pins.h"
#include "zarduino/module/oled.h"
#include "zarduino/module/mpu6050.h"
#include "zarduino/module/radio.h"

#include "control.h"
#include "buffer.h"

// Used defines for pins to save on memory usage

#define MOTOR_RIGHT_PWM PIN_TIMER1_A
#define MOTOR_RIGHT_DIR PIN_PD6
#define MOTOR_RIGHT_FEEDBACK PIN_INT0
#define MOTOR_LEFT_PWM PIN_TIMER1_B
#define MOTOR_LEFT_DIR PIN_PD5
#define MOTOR_LEFT_FEEDBACK PIN_INT1

#define BUTTON_1_PIN PIN_PC1
#define BUTTON_2_PIN PIN_PC2
#define BUTTON_3_PIN PIN_PC3
#define LED_PIN PIN_PD4
#define ADC_PIN PIN_PC0

#define RADIO_CSN_PIN PIN_PD7
#define RADIO_CE_PIN PIN_PB0

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
