#include "robot.h"

#include "zarduino/timing/timing.h"
#include "zarduino/comms/i2c.h"
#include "zarduino/comms/spi.h"
#include "zarduino/timing/delay.h"

#include "motors.h"
#include "interface.h"
#include "control.h"

#include <math.h>
#include <avr/interrupt.h>

Robot robot_create(void)
{
    Robot robot;
    robot.motor_right_pwm = PIN_TIMER1_A; //PB1
    robot.motor_right_dir = PIN_PD6;
    robot.motor_right_feedback = PIN_INT0; //PD2
    robot.motor_left_pwm = PIN_TIMER1_B; //PB2
    robot.motor_left_dir = PIN_PD5;
    robot.motor_left_feedback = PIN_INT1; //PD3

    robot.button_1_pin = PIN_PC1;
    robot.button_2_pin = PIN_PC2;
    robot.button_3_pin = PIN_PC3;
    robot.led_pin = PIN_PD4;
    robot.adc_pin = PIN_PC0;

    robot.radio_csn_pin = PIN_PD7;
    robot.radio_ce_pin = PIN_PB0;

    return robot;
}

void robot_init(Robot *robot)
{
    motors_init(robot);

    I2CConfig i2c_config = i2c_create_config();
    i2c_init_master(&i2c_config);

    SPIConfig spi_config = spi_create_config();
    spi_init_master(&spi_config);

    interface_init(robot);

    robot->mpu6050_config = mpu6050_create_config();
    mpu6050_init(&robot->mpu6050_config);

    robot->radio_config = radio_create_config();
    robot->radio_config.CSN = robot->radio_csn_pin;
    robot->radio_config.CE = robot->radio_ce_pin;
    robot->radio_config.IRQ = 0;

    robot->radio_config.rx_base_address = 0xA0000000;
    robot->radio_config.rx_pipe_addresses[0] = 0x12;
    robot->radio_config.rx_payload_sizes[0] = 5;

    radio_init_as_receiver(&robot->radio_config);
    delay(10);
    radio_start(&robot->radio_config);

    timer0_init_as_timer_accurate();

    robot->control_state = control_create();
}

void robot_loop(Robot *robot)
{
    const float v_sensitivity = 0.5/512.0;
    const float omega_sensitivity = 6.0/513.0;
    RadioRxStatus rx_status;
    uint8_t rx_payload[5];
    rx_status = radio_read_rx(&robot->radio_config, rx_payload, 5);
    if (rx_status != RADIO_RX_STATUS_NOT_USED &&
        rx_status != RADIO_RX_STATUS_EMPTY)
    {
        int16_t vert = rx_payload[0] | rx_payload[1]<<8;
        int16_t horiz = rx_payload[2] | rx_payload[3]<<8;
        robot->control_state.v_cmd = (float)vert * v_sensitivity;
        robot->control_state.omega_cmd = (float)horiz * omega_sensitivity;
    }

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

