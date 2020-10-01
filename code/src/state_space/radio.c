#include "robot.h"
#include "constants.h"
#include "config.h"
#include "interface_radio.h"
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

    interface_radio_init(&robot->radio_config);

    radio_init_as_receiver(&robot->radio_config);
    delay(10);
    radio_start(&robot->radio_config);

    robot->params[A1] = -900;
    robot->params[A2] = -100;
    robot->params[A3] = -180;
    robot->params[B4] = 40;
}

void robot_loop_active(RobotHandle robot_handle)
{
    Robot *robot = robot_handle;

    interface_radio_read_commands(&robot->radio_config, robot->cmd);

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
