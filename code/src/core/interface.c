#include "interface.h"

#include "zarduino/module/oled.h"
#include "zarduino/core/adc.h"
#include "zarduino/core/interrupt.h"
#include "zarduino/comms/uart.h"
#include "zarduino/timing/delay.h"

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

volatile uint8_t button_3_pressed;
Pin button_3_pin;
void button_3_callback(void)
{
    if (gpio_read(button_3_pin)) {
        button_3_pressed = 1;
    }
}

void interface_init(Robot *robot)
{
    UartConfig uart_config = uart_create_config();
    uart_config.baud_rate = 57600;
    uart_init(&uart_config);

    robot->oled_config = oled_create_config();
    oled_init(&robot->oled_config);

    ADCConfig adc_config = adc_create_config();
    adc_config.reference = ADC_REFERENCE_AVCC;
    adc_initialise(&adc_config);

    button_1_pin = robot->button_1_pin;
    button_2_pin = robot->button_2_pin;
    button_3_pin = robot->button_3_pin;

    gpio_mode_input_pullup(robot->button_1_pin);
    gpio_mode_input_pullup(robot->button_2_pin);
    gpio_mode_input_pullup(robot->button_3_pin);
    gpio_mode_output(robot->led_pin);
    gpio_write(robot->led_pin, 0);

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
    interrupt_pin_add_callback(
        robot->button_3_pin,
        button_3_callback 
    );
}

#define LINE_BUF_SIZE 50
char line[LINE_BUF_SIZE];

void interface_update_state(Robot *robot)
{
    if (button_3_pressed) {
        button_3_pressed = 0;
        robot->state = (robot->state + 1) % ROBOT_STATE_COUNT;
        // LED on with active states, which are odd states
        gpio_write(robot->led_pin, robot->state % 2);
        // Reset timer
        robot->seconds = 0;
    }


    switch (robot->state) {
        case ROBOT_STATE_PRE_CONTROL:
            snprintf(line, LINE_BUF_SIZE, "IDLE\n");
            break;
        case ROBOT_STATE_CONTROL:
            snprintf(line, LINE_BUF_SIZE, "CONTROL LOOP\n");
            break;
        case ROBOT_STATE_PRE_PASSIVE:
            snprintf(line, LINE_BUF_SIZE, "PRE PASSIVE\n");
            break;
        case ROBOT_STATE_PASSIVE:
            snprintf(line, LINE_BUF_SIZE, "PASSIVE TEST\n");
            break;
        case ROBOT_STATE_PRE_MOTOR:
            snprintf(line, LINE_BUF_SIZE, "PRE MOTOR\n");
            break;
        case ROBOT_STATE_MOTOR:
            snprintf(line, LINE_BUF_SIZE, "MOTOR TEST\n");
            break;
        default:
            break;
    }
    oled_print_string(&robot->oled_config, line);
}

void interface_update_data(Robot *robot)
{
    if (robot->state % 2) {
        // For active states, display time and write data to serial

        snprintf(line, LINE_BUF_SIZE, "SECONDS: %lu\n", (uint32_t)robot->seconds);
        oled_print_string(&robot->oled_config, line);

        uart_write_int32(robot->seconds * 10000);
        uart_write_int32(robot->control_state.theta * 10000);
        uart_write_int32(robot->control_state.theta_dot * 10000);
        uart_write_int32(robot->control_state.phi_dot * 10000);
        uart_write_int32(robot->control_state.psi_1_dot * 10000);
        uart_write_int32(robot->control_state.psi_2_dot * 10000);
        uart_write_int32(robot->control_state.motor_left_input * 10000);
        uart_write_int32(robot->control_state.motor_right_input * 10000);
    }
}

void interface_update_control(Robot *robot)
{
    static float start_value = 0;
    static float start_adc = 0;
    static uint8_t edit_value = 0;
    static uint8_t param_sel = 0;
    float adc_input;

    adc_input = ((float)(adc_read_wait(robot->adc_pin)>>2)) / 256.0;

    if (button_1_pressed) {
        button_1_pressed = 0;
        edit_value = 0;
        param_sel = (param_sel + 1) % 4;
    }

    if (button_2_pressed) {
        button_2_pressed = 0;

        if (!edit_value) {
            edit_value = 1;
            switch (param_sel) {
                case 0:
                    start_value = robot->control_state.controller_pid.kp;
                    break;
                case 1:
                    start_value = robot->control_state.controller_pid.ki;
                    break;
                case 2:
                    start_value = robot->control_state.controller_pid.kd;
                    break;
                case 3:
                    start_value = robot->control_state.controller_pid.kie_limit;
                    break;
            }
            start_adc = ((float)(adc_read_wait(robot->adc_pin)>>2)) / 256.0;
        } else {
            edit_value = 0;
        }
    }

    switch (param_sel) {
        case 0:
            if (edit_value)
                robot->control_state.controller_pid.kp =
                    start_value + (adc_input-start_adc)*2;
            snprintf(
                line, LINE_BUF_SIZE, "KP = %d\n",
                (int16_t)(robot->control_state.controller_pid.kp * 1000)
            );
            break;
        case 1:
            if (edit_value)
                robot->control_state.controller_pid.ki =
                    start_value + (adc_input-start_adc)*2;
            snprintf(
                line, LINE_BUF_SIZE, "KI = %d\n",
                (int16_t)(robot->control_state.controller_pid.ki * 1000)
            );
            break;
        case 2:
            if (edit_value)
                robot->control_state.controller_pid.kd =
                    start_value + (adc_input-start_adc)*2;
            snprintf(
                line, LINE_BUF_SIZE, "KD = %d\n",
                (int16_t)(robot->control_state.controller_pid.kd * 1000)
            );
            break;
        case 3:
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

    snprintf(
        line, LINE_BUF_SIZE, "CMD: %d %d\n",
        (int16_t)(robot->control_state.v_cmd * 1000),
        (int16_t)(robot->control_state.omega_cmd * 1000)
    );
    oled_print_string(&robot->oled_config, line);
}

void interface_update(Robot *robot)
{
    oled_clear(&robot->oled_config);

    interface_update_state(robot);
    interface_update_data(robot);

    // control_update_interface(robot);
}
