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

#define LINE_BUF_SIZE 50
void interface_update(Robot *robot, float dt)
{
    oled_clear(&robot->oled_data);

    static char line[LINE_BUF_SIZE];

    static float seconds;
    seconds += dt;
    snprintf(line, LINE_BUF_SIZE, "SECONDS: %lu\n", (uint32_t)seconds);
    oled_print_string(&robot->oled_config, &robot->oled_data, line);

    snprintf(
        line, LINE_BUF_SIZE, "PSI: %d %d\n",
        (int16_t)(robot->control_state.psi_1_dot*57.3),
        (int16_t)(robot->control_state.psi_2_dot*57.3)
    );
    oled_print_string(&robot->oled_config, &robot->oled_data, line);

    snprintf(
        line, LINE_BUF_SIZE, "THETA: %d\n",
        (int16_t)(robot->control_state.theta*57.3)
    );
    oled_print_string(&robot->oled_config, &robot->oled_data, line);

    snprintf(
        line, LINE_BUF_SIZE, "THETA_DOT: %d\n",
        (int16_t)(robot->control_state.theta_dot*57.3)
    );
    oled_print_string(&robot->oled_config, &robot->oled_data, line);

    snprintf(
        line, LINE_BUF_SIZE, "PHI_DOT: %d\n",
        (int16_t)(robot->control_state.phi_dot*57.3)
    );
    oled_print_string(&robot->oled_config, &robot->oled_data, line);

    static uint8_t edit_value = 0;
    static uint8_t param_sel = 0;
    float adc_input = ((float)adc_read_wait()) / 1024.0;
    switch (param_sel) {
        case 1:
            if (edit_value)
                robot->control_state.controller_pid.kp = adc_input*5;
            snprintf(
                line, LINE_BUF_SIZE, "KP = %u\n",
                (uint16_t)(robot->control_state.controller_pid.kp * 1000)
            );
            break;
        case 2:
            if (edit_value)
                robot->control_state.controller_pid.ki = adc_input*5;
            snprintf(
                line, LINE_BUF_SIZE, "KI = %u\n",
                (uint16_t)(robot->control_state.controller_pid.ki * 1000)
            );
            break;
        case 3:
            if (edit_value)
                robot->control_state.controller_pid.kd = adc_input*5;
            snprintf(
                line, LINE_BUF_SIZE, "KD = %u\n",
                (uint16_t)(robot->control_state.controller_pid.kd * 1000)
            );
            break;
        case 4:
            if (edit_value)
                robot->control_state.controller_pid.kie_limit = adc_input*20;
            snprintf(
                line, LINE_BUF_SIZE, "KIE_LIM = %u\n",
                (uint16_t)(robot->control_state.controller_pid.kie_limit * 1000)
            );
            break;
        default:
            line[0] = 0;
            break;
    }
    oled_print_string(&robot->oled_config, &robot->oled_data, line);

    oled_update(&robot->oled_config, &robot->oled_data);

    if (button_1_pressed) {
        button_1_pressed = 0;

        if (param_sel == 4)
            param_sel = 0;
        else
            param_sel++;
        gpio_write(robot->led_red_pin, 1);
    }
    if (button_2_pressed) {
        button_2_pressed = 0;

        edit_value = !edit_value;
        gpio_write(robot->led_red_pin, 0);
    }
}
