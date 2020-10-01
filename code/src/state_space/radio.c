#include "robot.h"
#include "constants.h"
#include "config.h"
#include "zarduino/comms/spi.h"
#include "zarduino/module/radio.h"
#include "zarduino/timing/delay.h"
#include <stdlib.h>
#include <string.h>

#define A1 0
#define A2 1
#define A3 2
#define B4 3
#define PARAM_COUNT 4

#define X_THETA 0
#define X_THETA_DOT 1
#define X_VEL 2
#define X_OMEGA 3
#define STATE_COUNT 4

#define CMD_VEL 0
#define CMD_OMEGA 1
#define CMD_COUNT 2

typedef struct {
    RobotBase base;
    RadioConfig radio_config;
    float params[PARAM_COUNT];
    float x[STATE_COUNT];
    float cmd[CMD_COUNT];
} Robot;

RobotHandle robot_create(void)
{
    Robot *robot = malloc(sizeof(Robot));
    memset(robot, 0, sizeof(*robot));
    return robot;
}

void robot_init(RobotHandle robot_handle)
{
    Robot *robot = robot_handle;

    SPIConfig spi_config = spi_create_config();
    spi_init_master(&spi_config);

    robot->radio_config = radio_create_config();
    robot->radio_config.CSN = RADIO_CSN_PIN;
    robot->radio_config.CE = RADIO_CE_PIN;
    robot->radio_config.rx_base_address = 0xA0000000;
    robot->radio_config.rx_pipe_addresses[0] = 0x12;
    robot->radio_config.rx_payload_sizes[0] = 4;

    radio_init_as_receiver(&robot->radio_config);
    delay(10);
    radio_start(&robot->radio_config);

    robot->params[A1] = -750;
    robot->params[A2] = -180;
    robot->params[A3] = -300;
    robot->params[B4] = -50;
}

void robot_loop_active(RobotHandle robot_handle)
{
    Robot *robot = robot_handle;

    static RadioRxStatus rx_status;
    static uint8_t data_in[4];
    static size_t radio_counter = 0;
    if (++radio_counter == 10) {
        radio_counter = 0;
        rx_status = radio_read_rx(&robot->radio_config, data_in, 4);
        if (rx_status != RADIO_RX_STATUS_NOT_USED &&
            rx_status != RADIO_RX_STATUS_EMPTY)
        {
            robot->cmd[CMD_VEL] = ((int16_t)(data_in[0] | data_in[1]<<8))/1024.0;
            robot->cmd[CMD_OMEGA] = ((int16_t)(data_in[2] | data_in[3]<<8))/1024.0;
        }
    }

    robot->x[X_THETA] = robot->base.y[Y_THETA];
    robot->x[X_THETA_DOT] = robot->base.y[Y_THETA_DOT];
    robot->x[X_VEL] = get_velocity(robot->base.y) - robot->cmd[CMD_VEL];
    robot->x[X_OMEGA] = robot->base.y[Y_PHI_DOT] - robot->cmd[CMD_OMEGA];

    robot->base.u[U_1] = - (
        robot->x[0] * robot->params[A1]*ETA_1_inv +
        robot->x[1] * robot->params[A2]*ETA_1_inv +
        robot->x[2] * robot->params[A3]*ETA_1_inv +
        robot->x[3] * robot->params[B4]*ETA_1_inv
    );

    robot->base.u[U_2] = - (
        robot->x[0] * robot->params[A1]*ETA_2_inv +
        robot->x[1] * robot->params[A2]*ETA_2_inv +
        robot->x[2] * robot->params[A3]*ETA_2_inv -
        robot->x[3] * robot->params[B4]*ETA_2_inv
    );
}

void robot_loop_inactive(RobotHandle robot_handle)
{
    // Do nothing
}
