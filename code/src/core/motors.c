#include "motors.h"
// #include "buffer.h"

#include "zarduino/timing/timing.h"
#include "zarduino/core/interrupt.h"
#include <stdint.h>

int16_t psi_left_count = 0;
int8_t psi_left_dir = 1;
// Buffer buffer_left;
void psi_left_callback(void)
{
    psi_left_count += psi_left_dir;
}

int16_t psi_right_count = 0;
int8_t psi_right_dir = 1;
// Buffer buffer_right;
void psi_right_callback(void)
{
    psi_right_count += psi_right_dir;
}

// const size_t BUFFER_N = 5;
// const float AVERAGE_WINDOW[5] = {0.2, 0.2, 0.2, 0.2, 0.2};

void motors_init(Robot *robot)
{
    gpio_mode_output(robot->motor_left_pwm);
    gpio_mode_output(robot->motor_left_dir);
    gpio_mode_input(robot->motor_left_feedback);

    gpio_mode_output(robot->motor_right_pwm);
    gpio_mode_output(robot->motor_right_dir);
    gpio_mode_input(robot->motor_right_feedback);

    timer1_init_as_pwm();
    // Need to set duty cycle here for some reason
    timer1_set_duty_cycle_a(0);
    timer1_set_duty_cycle_b(0);

    interrupt_external_add_callback(
        INTERRUPT_EXTERNAL_0,
        INTERRUPT_TYPE_ANY,
        psi_right_callback
    );
    interrupt_external_add_callback(
        INTERRUPT_EXTERNAL_1,
        INTERRUPT_TYPE_ANY,
        psi_left_callback
    );

    // buffer_right = buffer_create(BUFFER_N);
    // buffer_left = buffer_create(BUFFER_N);
}

void motors_set_cmd_right(Robot *robot, float cmd_right)
{
    // Motor PWM is active low
    if (cmd_right > 0) {
        if (cmd_right > 1)
            cmd_right = 1;
        timer1_set_duty_cycle_b(1 - cmd_right);
        gpio_write(robot->motor_right_dir, 0);
        psi_right_dir = 1;

    } else if(cmd_right < 0) {
        if (cmd_right < -1)
            cmd_right = -1;
        timer1_set_duty_cycle_b(1 + cmd_right);
        gpio_write(robot->motor_right_dir, 1);
        psi_right_dir = -1;

    } else {
        timer1_set_duty_cycle_b(0.999);
    }
}

void motors_set_cmd_left(Robot *robot, float cmd_left)
{
    // Motor PWM is active low
    if (cmd_left > 0) {
        if (cmd_left > 1)
            cmd_left = 1;
        timer1_set_duty_cycle_a(1 - cmd_left);
        gpio_write(robot->motor_left_dir, 1);
        psi_left_dir = 1;

    } else if(cmd_left < 0) {
        if (cmd_left < -1)
            cmd_left = -1;
        timer1_set_duty_cycle_a(1 + cmd_left);
        gpio_write(robot->motor_left_dir, 0);
        psi_left_dir = -1;

    } else {
        timer1_set_duty_cycle_a(0.999);
    }
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

    // buffer_set(&buffer_right, (float)psi_right_count / (dt*68.755));
    // buffer_set(&buffer_left, (float)psi_left_count / (dt*68.755));
    psi_right_count = 0;
    psi_left_count = 0;

    robot->state.psi_left_dot = 0;
    robot->state.psi_right_dot = 0;
    // robot->state.psi_right_dot = buffer_convolve_window(
    //     &buffer_right, AVERAGE_WINDOW
    // );
    // robot->state.psi_left_dot = buffer_convolve_window(
    //     &buffer_left, AVERAGE_WINDOW
    // );
}
