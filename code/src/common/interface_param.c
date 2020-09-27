#include "interface_param.h"

#include "zarduino/core/adc.h"
#include "zarduino/core/interrupt.h"
#include "zarduino/timing/delay.h"
#include "config.h"
#include <stdio.h>
#include <math.h>

#define INTERFACE_LINE_SIZE 16

size_t param_count = 0;
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

void interface_param_init(OLEDConfig *oled_config, float initial_param)
{
    oled_init(oled_config);

    ADCConfig adc_config = adc_create_config();
    adc_config.reference = ADC_REFERENCE_AVCC;
    adc_initialise(&adc_config);

    gpio_mode_input_pullup(BUTTON_1_PIN);
    gpio_mode_input_pullup(BUTTON_2_PIN);

    interrupt_pin_add_callback(
        BUTTON_1_PIN,
        button_1_callback
    );
    interrupt_pin_add_callback(
        BUTTON_2_PIN,
        button_2_callback
    );

    param_value = initial_param;
}

void interface_param_update(
    OLEDConfig *oled_config,
    float *param_values,
    const char **param_names,
    size_t param_count,
    float adc_scale)
{
    static float start_value = 0;
    static float start_adc = 0;
    static uint8_t edit_value = 0;
    static size_t param_i = 0;

    if (button_1_pressed) {
        button_1_pressed = 0;
        edit_value = 0;
        param_i = (param_i + 1) % param_count;
        param_value = param_values[param_i];
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
        param_value = start_value + (adc_input - start_adc)*adc_scale;
        param_values[param_i] = param_value;

    }

    static char line[INTERFACE_LINE_SIZE];
    if (param_value>0) {
        snprintf(
            line,
            INTERFACE_LINE_SIZE,
            "%s: %d.%d\n",
            param_names[param_i],
            (int16_t)(floorf(param_value)),
            (int16_t)(floorf((param_value-floorf(param_value)) * 1000))
        );
    } else {
        snprintf(
            line,
            INTERFACE_LINE_SIZE,
            "%s: -%d.%d\n",
            param_names[param_i],
            (int16_t)(floorf(-param_value)),
            (int16_t)(floorf((-param_value-floorf(-param_value)) * 1000))
        );
    }

    oled_clear(oled_config);
    oled_print_string(oled_config, line);
}
