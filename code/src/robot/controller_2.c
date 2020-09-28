#include "robot.h"
#include "constants.h"
#include "interface_param.h"
#include "zarduino/module/oled.h"
#include <stdlib.h>
#include <string.h>

#define PARAM_COUNT 7
#define A1 0
#define A2 1
#define A3 2
#define B4 3
#define L11 4
#define L22 5
#define L51 6

#define X_THETA 0
#define X_THETA_DOT 1
#define X_V 2
#define X_OMEGA 3
#define X_BIAS 4

typedef struct {
    RobotBase base;
    OLEDConfig oled_config;
    uint8_t asdf;
    float params[PARAM_COUNT];
    float x[5];
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
    robot->oled_config = oled_create_config();

    robot->params[A1] = -750;
    robot->params[A2] = -180;
    robot->params[A3] = -41;
    robot->params[B4] = 0;
    robot->params[L11] = 40;
    robot->params[L22] = 40;
    robot->params[L51] = 10;

    interface_param_init(&robot->oled_config, robot->params[A1]);
}

const char *param_names[PARAM_COUNT] = {
    "A1", "A2", "A3", "B4", "L11", "L22", "L51"
};

void robot_loop_active(RobotHandle robot_handle)
{
    Robot *robot = robot_handle;

    robot->x[X_V] = R*robot->base.y[Y_THETA_DOT] +
        (0.5*R*ETA_1)*robot->base.y[2] + (0.5*R*ETA_2)*robot->base.y[3];
    robot->x[X_OMEGA] =
        (0.5*R_D_ratio*ETA_1)*robot->base.y[2] - (0.5*R_D_ratio*ETA_2)*robot->base.y[Y_PSI_2_DOT];

    robot->x[X_THETA] += (
        robot->params[L11] * (robot->base.y[Y_THETA] - robot->x[X_THETA] - robot->x[X_BIAS])
        + robot->x[X_THETA_DOT]
    ) * robot->base.dt;

    robot->x[X_THETA_DOT] += (
        LAMBDA_2*robot->params[X_THETA]
        - 0.5*OMEGA_2*(ETA_1*robot->base.u[U_1] + ETA_2*robot->base.u[U_2])
        + robot->params[L22] * (robot->base.y[Y_THETA_DOT] - robot->x[X_THETA_DOT])
    ) * robot->base.dt;

    robot->x[X_BIAS] += robot->params[L51] * (
        robot->base.y[Y_THETA] - robot->x[X_THETA] - robot->x[X_BIAS]
    ) * robot->base.dt;

    robot->base.u[U_1] = - (
        robot->x[X_THETA] * robot->params[A1]*ETA_1_inv +
        robot->x[X_THETA_DOT] * robot->params[A2]*ETA_1_inv +
        robot->x[X_V] * robot->params[A3]*ETA_1_inv +
        robot->x[X_OMEGA] * robot->params[B4]*ETA_1_inv
    );

    robot->base.u[U_2] = - (
        robot->x[X_THETA] * robot->params[A1]*ETA_2_inv +
        robot->x[X_THETA_DOT] * robot->params[A2]*ETA_2_inv +
        robot->x[X_V] * robot->params[A3]*ETA_2_inv -
        robot->x[X_OMEGA] * robot->params[B4]*ETA_2_inv
    );
}

void robot_loop_inactive(RobotHandle robot_handle)
{
    Robot *robot = robot_handle;
    interface_param_update(
        &robot->oled_config,
        &robot->params[0],
        &param_names[0],
        PARAM_COUNT,
        10
    );
}
