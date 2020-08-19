#include "robot.h"

#include "zarduino/timing/timing.h"
#include "zarduino/comms/i2c.h"

#include "motors.h"
#include "interface.h"
#include "control.h"

#include <math.h>

Robot robot_create(void)
{
    Robot robot;
    robot.motor_right_pwm = PIN_TIMER1_A;
    robot.motor_right_dir = PIN_PD6;
    robot.motor_right_feedback = PIN_INT0;
    robot.motor_left_pwm = PIN_TIMER1_B;
    robot.motor_left_dir = PIN_PD5;
    robot.motor_left_feedback = PIN_INT1;

    robot.button_1_pin = PIN_PD1;
    robot.button_2_pin = PIN_PD0;
    robot.led_red_pin = PIN_PC3;

    return robot;
}

void robot_init(Robot *robot)
{
    motors_init(robot);

    I2CConfig i2c_config = i2c_create_config();
    i2c_init_master(&i2c_config);

    interface_init(robot);

    robot->mpu6050_config = mpu6050_create_config();
    mpu6050_init(&robot->mpu6050_config);
    robot->mpu6050_data = (MPU6050Data){};
    mpu6050_calibrate(&robot->mpu6050_config);

    timer0_init_as_timer_accurate();

    robot->control_state = control_create();

    // Temporary
    motors_set_left(robot, 0.3, 1);
    motors_set_right(robot, 0.3, 1);
}

void robot_loop(Robot *robot)
{
    static uint64_t current_ticks = 0;
    static uint64_t prev_ticks, delta_ticks;
    prev_ticks = current_ticks;
    current_ticks = timer0_accurate_get_ticks();
    delta_ticks = current_ticks - prev_ticks;
    float dt = ((float)delta_ticks) * 64e-9;

    motors_get_feedback(robot, dt);

    mpu6050_read_data(&robot->mpu6050_config, &robot->mpu6050_data);

    static float theta;
    theta = atan2(
        -robot->mpu6050_data.accel[0], robot->mpu6050_data.accel[2]
    );
    robot->control_state.theta = theta;
    robot->control_state.theta_dot = robot->mpu6050_data.gyro[1] * 0.01745;
    robot->control_state.phi_dot =
        cos(theta) * robot->mpu6050_data.gyro[2] * 0.01745
        - sin(theta) * robot->mpu6050_data.gyro[0] * 0.01745;

    robot->control_state.psi_1_dot = robot->motor_left_vel;
    robot->control_state.psi_2_dot = robot->motor_right_vel;

    control_update(&robot->control_state, dt);

    if (robot->control_state.motor_left_input > 1)
        robot->control_state.motor_left_input = 1;
    if (robot->control_state.motor_left_input < -1)
        robot->control_state.motor_left_input = -1;
    if (robot->control_state.motor_right_input > 1)
        robot->control_state.motor_right_input = 1;
    if (robot->control_state.motor_right_input < -1)
        robot->control_state.motor_right_input = -1;

    // Temporary
    robot->control_state.motor_left_input = 0.5;
    robot->control_state.motor_right_input = 0.5;

    if (robot->control_state.motor_left_input > 0)
        motors_set_left(robot, robot->control_state.motor_left_input, 1);
    else
        motors_set_left(robot, -robot->control_state.motor_left_input, 0);

    if (robot->control_state.motor_right_input > 0)
        motors_set_right(robot, robot->control_state.motor_right_input, 1);
    else
        motors_set_right(robot, -robot->control_state.motor_right_input, 0);

    interface_update(robot, dt);
}

