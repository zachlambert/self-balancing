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
} Robot;

Robot robot_create(void)
{
    Robot robot;
    robot.motor_right_pwm = PIN_TIMER1_A;
    robot.motor_right_dir = PIN_PD6;
    robot.motor_right_feedback = PIN_INT0;
    robot.motor_left_pwm = PIN_TIMER1_B;
    robot.motor_left_dir = PIN_PD5;
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
        INTERRUPT_TYPE_ANY,
        int0_callback
    );
    interrupt_external_add_callback(
        INTERRUPT_EXTERNAL_1,
        INTERRUPT_TYPE_ANY,
        int1_callback
    );

    timer0_init_as_timer_accurate();
}

void robot_loop(Robot *robot)
{
    // Each motor cycle outputs 6 pulses -> 12 interrupts
    // Gear reduction of 45:1
    // Therefore, 45*12 = 540 interrupts per revolution
    // Count = num interrupts in dt
    // interrupts per sec = count / dt
    // rev per sec = (count/540) / dt
    //             = count / (dt*540)
    // rpm = rev per sec * 60
    //     = count / (dt*9)
    // BUT, for some reason this gave values 80% of the
    // measured values, so multiply by 1.25

    static uint64_t current_ticks = 0;
    static uint64_t prev_ticks, delta_ticks;
    prev_ticks = current_ticks;
    current_ticks = timer0_accurate_get_ticks();
    delta_ticks = current_ticks - prev_ticks;
    float dt = ((float)delta_ticks) * 64e-9;

    static float seconds = 0;
    seconds += dt;

    static float rpm_right, rpm_left;
    rpm_right = int0_count * 1.25f / (dt * 9);
    int0_count = 0;
    rpm_left = int1_count * 1.25f / (dt * 9);
    int1_count = 0;

    static size_t i = 0;
    i++;
    static char lines[3][30];
    snprintf(lines[0], 30, "SECONDS: %lu\n", (uint32_t)seconds);

    snprintf(lines[1], 30, "RIGHT RPM: %u\n", (uint16_t)rpm_right);
    snprintf(lines[2], 30, "LEFT RPM:  %u\n", (uint16_t)rpm_left);

    oled_clear(&robot->oled_data);
    for (size_t i = 0; i < 3; i++) {
        oled_print_string(
            &robot->oled_config,&robot->oled_data, lines[i]
        );
    }
    oled_update(&robot->oled_config, &robot->oled_data);

}

int main(void)
{
    Robot robot = robot_create();
    robot_init(&robot);

    set_left_motor(&robot, 0.3, 1);
    set_right_motor(&robot, 0.3, 1);

    while (1) {
        robot_loop(&robot);
        // Writing to the oled creates enough of a delay
        // in the loop.
        // Using a delay function seems to break the timer,
        // so if I later remove the oled code from the loop,
        // use a timer callback to run the loop at regular
        // intervals, but don't rely on it being accurate, so
        // still use the accurate timer for measuring the
        // time elapsed.
    }
}
