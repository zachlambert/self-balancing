#include "interface_common.h"

#include "zarduino/core/interrupt.h"
#include "config.h"

#define INTERFACE_LINE_SIZE 16

volatile uint8_t button_3_pressed = 0;
void button_3_callback(void)
{
    if (gpio_read(BUTTON_3_PIN)) {
        button_3_pressed = 1;
    }
}

void interface_common_init(void)
{
    gpio_mode_input_pullup(BUTTON_3_PIN);
    gpio_mode_output(LED_PIN);
    gpio_write(LED_PIN, 0);

    interrupt_pin_add_callback(
        BUTTON_3_PIN,
        button_3_callback
    );
}

void interface_common_update(RobotHandle robot_handle)
{
    RobotBase *robot = robot_handle;

    if (button_3_pressed) {
        button_3_pressed = 0;
        robot->active = !robot->active;
        gpio_write(LED_PIN, robot->active);
        robot->seconds = 0;
        if (!robot->active) {
            robot->pwm[PWM_1] = 0;
            robot->pwm[PWM_2] = 0;
        } else {
            mpu6050_calibrate(&robot->mpu6050_config);
        }
    }
}
