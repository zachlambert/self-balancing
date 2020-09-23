#include "interface.h"

#include "zarduino/module/oled.h"
#include "zarduino/core/adc.h"
#include "zarduino/core/interrupt.h"
#include "zarduino/comms/uart.h"
#include "zarduino/timing/delay.h"

volatile uint8_t button_1_pressed;
void button_1_callback(void)
{
    if (gpio_read(BUTTON_1_PIN)) {
        button_1_pressed = 1;
    }
}

volatile uint8_t button_2_pressed;
void button_2_callback(void)
{
    if (gpio_read(BUTTON_2_PIN)) {
        button_2_pressed = 1;
    }
}

volatile uint8_t button_3_pressed;
void button_3_callback(void)
{
    if (gpio_read(BUTTON_3_PIN)) {
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

    gpio_mode_input_pullup(BUTTON_1_PIN);
    gpio_mode_input_pullup(BUTTON_2_PIN);
    gpio_mode_input_pullup(BUTTON_3_PIN);
    gpio_mode_output(LED_PIN);
    gpio_write(LED_PIN, 0);

    button_1_pressed = 0;
    button_2_pressed = 0;

    interrupt_pin_add_callback(
        BUTTON_1_PIN,
        button_1_callback 
    );
    interrupt_pin_add_callback(
        BUTTON_2_PIN,
        button_2_callback 
    );
    interrupt_pin_add_callback(
        BUTTON_3_PIN,
        button_3_callback 
    );
}

void interface_send_state(Robot *robot)
{
    uart_write_int32(robot->seconds * 10000);
    uart_write_int32(robot->state.theta * 10000);
    uart_write_int32(robot->state.theta_dot * 10000);
    uart_write_int32(robot->state.phi_dot * 10000);
    uart_write_int32(robot->state.psi_right_dot * 10000);
    uart_write_int32(robot->state.psi_left_dot * 10000);
    uart_write_int32(robot->state.motor_cmd_right * 10000);
    uart_write_int32(robot->state.motor_cmd_left * 10000);
}

void interface_update_params(Robot *robot)
{
    static float start_value = 0;
    static float start_adc = 0;
    static uint8_t edit_value = 0;
    static size_t param_i = 0;

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
            start_adc = ((float)(adc_read_wait(ADC_PIN)>>2)) / 256.0;
        } else {
            edit_value = 0;
        }
    }

    if (edit_value) {
        float adc_input = ((float)(adc_read_wait(ADC_PIN)>>2)) / 256.0;
        controller_set_param(
            robot->controller_handle, param_i,
            start_value + (adc_input - start_adc)*2
        );
    }
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
        gpio_write(LED_PIN, robot->active);
        robot->seconds = 0;
        if (robot->active) {
            oled_clear(&robot->oled_config);
        } else {
            robot->state.motor_cmd_right = 0;
            robot->state.motor_cmd_left = 0;
        }
    }

    if (robot->active) {
        interface_send_state(robot);
    } else {
        oled_clear(&robot->oled_config);
        interface_update_params(robot);
    }
}
