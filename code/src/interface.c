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
    oled_clear(&robot->oled_config);

    static char line[LINE_BUF_SIZE];

    static float seconds;
    seconds += dt;
    snprintf(line, LINE_BUF_SIZE, "SECONDS: %lu\n", (uint32_t)seconds);
    oled_print_string(&robot->oled_config, line);

    snprintf(
        line, LINE_BUF_SIZE, "PSI: %d %d\n",
        (int16_t)(robot->control_state.psi_1_dot*57.3),
        (int16_t)(robot->control_state.psi_2_dot*57.3)
    );
    oled_print_string(&robot->oled_config, line);

    snprintf(
        line, LINE_BUF_SIZE, "THETA: %d\n",
        (int16_t)(robot->control_state.theta*57.3)
    );
    oled_print_string(&robot->oled_config, line);

    snprintf(
        line, LINE_BUF_SIZE, "THETA_DOT: %d\n",
        (int16_t)(robot->control_state.theta_dot*57.3)
    );
    oled_print_string(&robot->oled_config, line);

    snprintf(
        line, LINE_BUF_SIZE, "PHI_DOT: %d\n",
        (int16_t)(robot->control_state.phi_dot*57.3)
    );
    oled_print_string(&robot->oled_config, line);

    static float start_value = 0;
    static float start_adc = 0;
    static uint8_t edit_value = 0;
    static uint8_t param_sel = 0;
    static float adc_input;

    adc_input = ((float)(adc_read_wait()>>2)) / 256.0;

    if (button_1_pressed) {
        button_1_pressed = 0;

        edit_value = 0;
        if (param_sel == 4)
            param_sel = 0;
        else
            param_sel++;
    }
    if (button_2_pressed) {
        button_2_pressed = 0;

        if (!edit_value) {
            edit_value = 1;
            switch (param_sel) {
                case 1:
                    start_value = robot->control_state.controller_pid.kp;
                    break;
                case 2:
                    start_value = robot->control_state.controller_pid.ki;
                    break;
                case 3:
                    start_value = robot->control_state.controller_pid.kd;
                    break;
                case 4:
                    start_value = robot->control_state.controller_pid.kie_limit;
                    break;
            }
            start_adc = ((float)(adc_read_wait()>>2)) / 256.0;
        } else {
            edit_value = 0;
        }
    }

    switch (param_sel) {
        case 1:
            if (edit_value)
                robot->control_state.controller_pid.kp =
                    start_value + (adc_input-start_adc)*2;
            snprintf(
                line, LINE_BUF_SIZE, "KP = %d\n",
                (int16_t)(robot->control_state.controller_pid.kp * 1000)
            );
            break;
        case 2:
            if (edit_value)
                robot->control_state.controller_pid.ki =
                    start_value + (adc_input-start_adc)*2;
            snprintf(
                line, LINE_BUF_SIZE, "KI = %d\n",
                (int16_t)(robot->control_state.controller_pid.ki * 1000)
            );
            break;
        case 3:
            if (edit_value)
                robot->control_state.controller_pid.kd =
                    start_value + (adc_input-start_adc)*2;
            snprintf(
                line, LINE_BUF_SIZE, "KD = %d\n",
                (int16_t)(robot->control_state.controller_pid.kd * 1000)
            );
            break;
        case 4:
            if (edit_value)
                robot->control_state.controller_pid.kie_limit =
                    start_value + (adc_input-start_adc)*4;
            snprintf(
                line, LINE_BUF_SIZE, "KIE_LIM = %d\n",
                (int16_t)(robot->control_state.controller_pid.kie_limit * 1000)
            );
            break;
        default:
            line[0] = 0;
            break;
    }
    oled_print_string(&robot->oled_config, line);
}
