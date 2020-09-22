#include "interface.h"

#include "zarduino/module/oled.h"
#include "zarduino/core/adc.h"
#include "zarduino/core/interrupt.h"
#include "zarduino/comms/uart.h"
#include "zarduino/timing/delay.h"

#include <stdint.h>
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

void interface_send_state(Robot *robot)
{
    uart_write_int32(robot->seconds * 10000);
    uart_write_int32(robot->state.theta * 10000);
    uart_write_int32(robot->state.theta_dot * 10000);
    uart_write_int32(robot->state.phi_dot * 10000);
    uart_write_int32(robot->state.psi_1_dot * 10000);
    uart_write_int32(robot->state.psi_2_dot * 10000);
    uart_write_int32(robot->state.motor_cmd_1 * 10000);
    uart_write_int32(robot->state.motor_cmd_2 * 10000);
}

void interface_update_params(Robot *robot)
{
    static float start_value = 0;
    static float start_adc = 0;
    static uint8_t edit_value = 0;
    static size_t param_i = 0;

    float adc_input = ((float)(adc_read_wait(robot->adc_pin)>>2)) / 256.0;

    if (button_1_pressed) {
        button_1_pressed = 0;
        edit_value = 0;
        param_i = (param_i + 1) % CONTROLLER_PARAM_COUNT;
    }

    if (button_2_pressed) {
        button_2_pressed = 0;

        if (!edit_value) {
            edit_value = 1;
            start_value = controller_get_param(robot->controller_handle, param_i);
            start_adc = ((float)(adc_read_wait(robot->adc_pin)>>2)) / 256.0;
        } else {
            edit_value = 0;
        }
    }

    controller_set_param(
        robot->controller_handle, param_i,
        start_value + (adc_input - start_adc)*2
    );

    oled_print_string(
        &robot->oled_config,
        controller_get_string(robot->controller_handle, param_i)
    );
}

void interface_update(Robot *robot)
{
    if (button_3_pressed) {
        button_3_pressed = 0;
        robot->active = !robot->active;
        gpio_write(robot->led_pin, robot->active);
        robot->seconds = 0;
        if (robot->active) {
            oled_clear(&robot->oled_config);
            oled_print_string(&robot->oled_config, "ACTIVE\n");
        }
    }

    if (robot->active) {
        interface_send_state(robot);
    } else {
        oled_clear(&robot->oled_config);
        oled_print_string(&robot->oled_config, "INACTIVE\n");
        interface_update_params(robot);
    }
}
