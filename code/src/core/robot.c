#include "robot.h"

#include "zarduino/timing/timing.h"
#include "zarduino/comms/i2c.h"
#include "zarduino/comms/spi.h"
#include "zarduino/timing/delay.h"

#include "config.h"
#include "motors.h"
#include "interface.h"
#include "control.h"

#include <math.h>
#include <avr/interrupt.h>

void robot_init(Robot *robot)
{
    motors_init();

    I2CConfig i2c_config = i2c_create_config();
    i2c_init_master(&i2c_config);

    SPIConfig spi_config = spi_create_config();
    spi_init_master(&spi_config);

    interface_init(robot);

    robot->mpu6050_config = mpu6050_create_config();
    mpu6050_init(&robot->mpu6050_config);

    robot->radio_config = radio_create_config();
    robot->radio_config.CSN = RADIO_CSN_PIN;
    robot->radio_config.CE = RADIO_CE_PIN;

    robot->radio_config.rx_base_address = 0xA0000000;
    robot->radio_config.rx_pipe_addresses[0] = 0x12;
    robot->radio_config.rx_payload_sizes[0] = 4;

    radio_init_as_receiver(&robot->radio_config);
    delay(10);
    radio_start(&robot->radio_config);

    timer0_init_as_timer_accurate();

    robot->controller_handle = controller_init();
}

inline void robot_loop_radio(Robot *robot)
{
    const float v_sensitivity = 0.5/512.0;
    const float omega_sensitivity = 6.0/512.0;
    RadioRxStatus rx_status;
    uint8_t rx_payload[4];
    rx_status = radio_read_rx(&robot->radio_config, rx_payload, 4);
    if (rx_status != RADIO_RX_STATUS_NOT_USED &&
        rx_status != RADIO_RX_STATUS_EMPTY)
    {
        int16_t vert = rx_payload[0] | rx_payload[1]<<8;
        int16_t horiz = rx_payload[2] | rx_payload[3]<<8;
        robot->state.v_cmd = (float)vert * v_sensitivity;
        robot->state.omega_cmd = (float)horiz * omega_sensitivity;
    }
}

uint64_t current_ticks = 0;
uint64_t prev_ticks, delta_ticks;
inline void robot_loop_time(State *state)
{
    prev_ticks = current_ticks;
    current_ticks = timer0_accurate_get_ticks();
    delta_ticks = current_ticks - prev_ticks;
    state->dt = ((float)delta_ticks) * 64e-9;
    state->seconds += state->dt;
}

inline void robot_loop_sensors(Robot *robot)
{
    motors_get_feedback(&robot->state);
    mpu6050_read_data(&robot->mpu6050_config, &robot->mpu6050_data);

    robot->state.theta = atan2(
        -robot->mpu6050_data.accel[0], robot->mpu6050_data.accel[2]
    );
    robot->state.theta_dot = robot->mpu6050_data.gyro[1] * 0.01745;
    robot->state.phi_dot =
        cos(robot->state.theta) * robot->mpu6050_data.gyro[2] * 0.01745
        - sin(robot->state.theta) * robot->mpu6050_data.gyro[0] * 0.01745;
}

inline void robot_loop_actuate(State *state)
{
    motors_set_cmd_right(state);
    motors_set_cmd_left(state);
}

void robot_loop(Robot *robot)
{
    robot_loop_time(&robot->state);
    robot_loop_radio(robot);
    robot_loop_sensors(robot);

    if (robot->active) {
        controller_update(&robot->state, robot->controller_handle);
    }

    robot_loop_actuate(&robot->state);
    interface_update(robot);
}

