#include "robot.h"
#include "constants.h"
#include "config.h"
#include "zarduino/comms/spi.h"
#include "zarduino/module/radio.h"
#include "zarduino/timing/delay.h"
#include <stdlib.h>
#include <string.h>

#define PARAM_COUNT 4
#define A1 0
#define A2 1
#define A3 2
#define B4 3

typedef struct {
    RobotBase base;
    RadioConfig radio_config;
    float cmd[2];
    float params[PARAM_COUNT];
    float x[4];
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
    robot->params[A3] = -630;
    robot->params[B4] = 0;
}

void robot_loop_active(RobotHandle robot_handle)
{
    Robot *robot = robot_handle;

    static RadioRxStatus rx_status;
    static uint8_t data_in[4];
    rx_status = radio_read_rx(&robot->radio_config, data_in, 4);
    if (rx_status != RADIO_RX_STATUS_NOT_USED &&
        rx_status != RADIO_RX_STATUS_EMPTY)
    {
        robot->cmd[0] = (float)(data_in[0] | data_in[1]<<8)*(0.5/1024.0);
        robot->cmd[1] = (float)(data_in[2] | data_in[3]<<8)*(1.5/1024.0);
        delay(1000);
    }

    robot->x[0] = robot->base.y[0];
    robot->x[1] = robot->base.y[1];
    robot->x[2] = (0.5*R)*robot->base.y[2] + (0.5*R)*robot->base.y[3] - robot->cmd[0];
    robot->x[3] = (0.5*R_D_ratio)*robot->base.y[2] - (0.5*R_D_ratio)*robot->base.y[3] - robot->cmd[1];

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
