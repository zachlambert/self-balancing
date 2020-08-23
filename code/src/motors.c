#include "motors.h"

#include "zarduino/timing/timing.h"
#include "zarduino/core/interrupt.h"
#include <stdint.h>

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

void motors_init(Robot *robot)
{
    gpio_mode_output(robot->motor_left_pwm);
    gpio_mode_output(robot->motor_left_dir);
    gpio_mode_input(robot->motor_left_feedback);

    gpio_mode_output(robot->motor_right_pwm);
    gpio_mode_output(robot->motor_right_dir);
    gpio_mode_input(robot->motor_right_feedback);

    timer1_init_as_pwm();
    timer1_set_duty_cycle_a(0.95);
    timer1_set_duty_cycle_b(0.95);

    interrupt_external_add_callback(
        INTERRUPT_EXTERNAL_0,
        INTERRUPT_TYPE_ANY,
        int0_callback
    );
    interrupt_external_add_callback(
        INTERRUPT_EXTERNAL_1,
        INTERRUPT_TYPE_ANY,
        int1_callback
    );
}

void motors_set_left(Robot *robot, float duty_cycle, uint8_t dir)
{
    // Motor PWM is active low
    timer1_set_duty_cycle_a(1-duty_cycle);
    gpio_write(robot->motor_left_dir, dir);
}

void motors_set_right(Robot *robot, float duty_cycle, uint8_t dir)
{
    // Motor PWM is active low
    timer1_set_duty_cycle_b(1-duty_cycle);
    // Direction reversed
    gpio_write(robot->motor_right_dir, (~dir)&0x01);
}

void motors_get_feedback(Robot *robot, float dt)
{
    // Each motor cycle outputs 6 pulses -> 12 interrupts
    // Gear reduction of 45:1
    // Therefore, 45*12 = 540 interrupts per revolution
    // Count = num interrupts in dt
    // interrupts per sec = count / dt
    // rev per sec = (count/540) / dt
    //             = count / (dt*540)
    // To get rad/s, multiply by 2*pi
    // rad/s ~= count / (dt*85.944)
    // BUT, for some reason this gives values 80% of the
    // measured values, so multiply by 1.25
    // rad/s ~= count / (dt*68.755)

    if (int0_count > 0) {
        robot->motor_right_vel = int0_count / (dt*68.755);
    } else {
        robot->motor_right_vel = 0;
    }
    int0_count = 0;
    if (int1_count > 0) {
        robot->motor_left_vel = int1_count / (dt*68.755);
    } else {
        robot->motor_left_vel = 0;
    }
    int1_count = 0;
}
