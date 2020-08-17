#include "zarduino/core/pins.h"
#include "zarduino/timing/timing.h"
#include "zarduino/timing/delay.h"

#include "zarduino/comms/i2c.h"
#include "zarduino/module/oled.h"

#include "zarduino/core/interrupt.h"

#include <stdint.h>
#include <stdio.h>

typedef struct {
    Pin motor_left_pwm;
    Pin motor_left_dir;
    Pin motor_left_feedback;
    Pin motor_right_pwm;
    Pin motor_right_dir;
    Pin motor_right_feedback;

    OLEDConfig oled_config;
    OLEDData oled_data;

    float dt;
} Robot;

Robot robot_create(void)
{
    Robot robot;
    robot.motor_right_pwm = PIN_TIMER1_A;
    robot.motor_right_dir = PIN_PB7;
    robot.motor_right_feedback = PIN_INT0;
    robot.motor_left_pwm = PIN_TIMER1_B;
    robot.motor_left_dir = PIN_PB6;
    robot.motor_left_feedback = PIN_INT1;
    return robot;
}

void robot_init_motors(Robot *robot)
{
    gpio_mode_output(robot->motor_left_pwm);
    gpio_mode_output(robot->motor_left_dir);
    gpio_mode_input_pullup(robot->motor_left_feedback);

    gpio_mode_output(robot->motor_right_pwm);
    gpio_mode_output(robot->motor_right_dir);
    gpio_mode_input_pullup(robot->motor_right_feedback);

    timer1_init_as_pwm();
    timer1_set_duty_cycle_a(0.95);
    timer1_set_duty_cycle_b(0.95);
}

void set_left_motor(Robot *robot, float duty_cycle, uint8_t dir)
{
    // Motor PWM is active low
    timer1_set_duty_cycle_a(1-duty_cycle);
    gpio_write(robot->motor_left_dir, dir);
}

void set_right_motor(Robot *robot, float duty_cycle, uint8_t dir)
{
    // Motor PWM is active low
    timer1_set_duty_cycle_b(1-duty_cycle);
    // Direction reversed
    gpio_write(robot->motor_right_dir, (~dir)&0x01);
}

size_t int0_count = 0;
void int0_callback(void)
{
    int0_count++;
}

size_t int1_count = 0;
void int1_callback(void)
{
    int1_count++;
}

void robot_init(Robot *robot)
{
    robot_init_motors(robot);

    I2CConfig i2c_config = i2c_create_config();
    i2c_init_master(&i2c_config);

    robot->oled_config = oled_create_config();
    oled_init(&robot->oled_config);
    robot->oled_data = oled_create_data(&robot->oled_config);

    interrupt_external_add_callback(
        INTERRUPT_EXTERNAL_0,
        INTERRUPT_TYPE_RISING,
        int0_callback
    );
    interrupt_external_add_callback(
        INTERRUPT_EXTERNAL_1,
        INTERRUPT_TYPE_RISING,
        int1_callback
    );

    robot->dt = 100; // 100 ms
}

Robot robot;
void robot_callback(void)
{
    static char lines[5][30];

    snprintf(lines[0], 30, "INT0 %u\n", int0_count);
    snprintf(lines[1], 30, "INT1 %u\n", int1_count);
    oled_clear(&robot.oled_data);
    for (size_t i = 0; i < 2; i++) {
        oled_print_string(
            &robot.oled_config,&robot.oled_data, lines[i]
        );
    }
    oled_update(&robot.oled_config, &robot.oled_data);

    int0_count = 0;
    int1_count = 0;
}


void robot_start(Robot *robot)
{
    timer0_init_as_timer_ms(robot->dt, robot_callback);
}

int main(void)
{
    robot = robot_create();
    robot_init(&robot);

    set_left_motor(&robot, 0.3, 1);
    set_right_motor(&robot, 0.3, 1);

    robot_start(&robot);
    while (1) {}
}
