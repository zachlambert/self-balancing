#include "interface_param.h"

#include "zarduino/module/oled.h"
#include "zarduino/core/adc.h"
#include "zarduino/core/interrupt.h"
#include "zarduino/comms/uart.h"
#include "zarduino/timing/delay.h"
#include "config.h"

#define INTERFACE_LINE_SIZE 16

float param_value = 0;

volatile uint8_t button_1_pressed = 0;
void button_1_callback(void)
{
    if (gpio_read(BUTTON_1_PIN)) {
        button_1_pressed = 1;
    }
}

volatile uint8_t button_2_pressed = 0;
void button_2_callback(void)
{
    if (gpio_read(BUTTON_2_PIN)) {
        button_2_pressed = 1;
    }
}

void interface_init(RobotHandle robot_handle)
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

    param_value = controller_get_param(&robot->state, 0);
}

void interface_send_state(State *state)
{
    uart_write_int32(state->seconds * 10000);
    uart_write_int32(state->theta * 10000);
    uart_write_int32(state->theta_dot * 10000);
    uart_write_int32(state->phi_dot * 10000);
    uart_write_int32(state->psi_right_dot * 10000);
    uart_write_int32(state->psi_left_dot * 10000);
    uart_write_int32(state->motor_cmd_right * 10000);
    uart_write_int32(state->motor_cmd_left * 10000);
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
        param_value = controller_get_param(robot->controller_handle, param_i);
    }

    if (button_2_pressed) {
        button_2_pressed = 0;

        if (!edit_value) {
            edit_value = 1;
            start_value = param_value;
            start_adc = ((float)(adc_read_wait(ADC_PIN)>>2)) / 256.0;
        } else {
            edit_value = 0;
        }
    }

    if (edit_value) {
        float adc_input = ((float)(adc_read_wait(ADC_PIN)>>2)) / 256.0;
        param_value = start_value + (adc_input - start_adc)*2;
        controller_set_param(robot->controller_handle, param_i, param_value);

    }

    static char line[INTERFACE_LINE_SIZE];
    snprintf(
        line,
        INTERFACE_LINE_SIZE,
        "%s: %d\n",
        controller_get_param_name(param_i),
        (int16_t)(param_value*1000)
    );

    oled_clear(&robot->oled_config);
    oled_print_string(
        &robot->oled_config,
        line
    );
}

void interface_update(Robot *robot)
{
    if (button_3_pressed) {
        button_3_pressed = 0;
        robot->active = !robot->active;
        gpio_write(LED_PIN, robot->active);
        robot->state.seconds = 0;
        if (robot->active) {
            oled_clear(&robot->oled_config);
            oled_print_string(&robot->oled_config, "\n");
        } else {
            robot->state.motor_cmd_right = 0;
            robot->state.motor_cmd_left = 0;
        }
    }

    if (robot->active) {
        interface_send_state(&robot->state);
    } else {
        interface_update_params(robot);
    }
}
