#include "interface.h"

#include "zarduino/module/oled.h"
#include "zarduino/core/adc.h"
#include "zarduino/core/interrupt.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <avr/interrupt.h>

volatile uint8_t button_1_pressed;
Pin button_1_pin;
void button_1_callback(void)
{
    if (gpio_read(button_1_pin)) {
        button_1_pressed = 1;
    }
}

volatile uint8_t button_2_pressed;
Pin button_2_pin;
void button_2_callback(void)
{
    if (gpio_read(button_2_pin)) {
        button_2_pressed = 1;
    }
}

void interface_init(Robot *robot)
{
    robot->oled_config = oled_create_config();
    oled_init(&robot->oled_config);
    robot->oled_data = oled_create_data(&robot->oled_config);

    ADCConfig config = adc_create_config();
    adc_initialise(&config);

    button_1_pin = robot->button_1_pin;
    button_2_pin = robot->button_2_pin;

    gpio_mode_input_pullup(robot->button_1_pin);
    gpio_mode_input_pullup(robot->button_2_pin);
    gpio_mode_output(robot->led_red_pin);
    gpio_write(robot->led_red_pin, 0);

    button_1_pressed = 0;
    button_2_pressed = 0;

    interrupt_pin_add_callback(
        robot->button_1_pin,
        button_1_callback 
    );
    interrupt_pin_add_callback(
        robot->button_2_pin,
        button_2_callback 
    );
}

void interface_update(Robot *robot, float dt)
{
    static char lines[6][30];

    static float seconds;
    seconds += dt;
    snprintf(lines[0], 30, "SECONDS: %lu\n", (uint32_t)seconds);

    snprintf(
        lines[1], 30, "RIGHT RPM: %u\n",
        (uint16_t)(robot->motor_left_vel*9.549)
    );
    snprintf(
        lines[2], 30, "LEFT RPM:  %u\n",
        (uint16_t)(robot->motor_right_vel*9.549)
    );
    snprintf(
        lines[3], 30, "ACC: %d %d %d\n",
        (int16_t)(robot->mpu6050_data.accel[0]),
        (int16_t)(robot->mpu6050_data.accel[1]),
        (int16_t)(robot->mpu6050_data.accel[2])
    );
    snprintf(
        lines[4], 30, "GYRO: %d %d %d\n",
        (int16_t)(robot->mpu6050_data.gyro[0]*57.3),
        (int16_t)(robot->mpu6050_data.gyro[1]*57.3),
        (int16_t)(robot->mpu6050_data.gyro[2]*57.3)
    );
    snprintf(
        lines[5], 30, "ADC: %u\n", adc_read_wait()
    );

    oled_clear(&robot->oled_data);
    for (size_t i = 0; i < 6; i++) {
        oled_print_string(
            &robot->oled_config,&robot->oled_data, lines[i]
        );
    }
    oled_update(&robot->oled_config, &robot->oled_data);

    if (button_1_pressed) {
        button_1_pressed = 0;
        gpio_write(robot->led_red_pin, 1);
    }
    if (button_2_pressed) {
        button_2_pressed = 0;
        gpio_write(robot->led_red_pin, 0);
    }
}
