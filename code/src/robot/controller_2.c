#include "robot.h"
#include "constants.h"
#include "interface_param.h"
#include "zarduino/module/oled.h"
#include <stdlib.h>
#include <string.h>

#define PARAM_COUNT 6
#define A1 0
#define A2 1
#define A3 2
#define B4 3
#define L1 4
#define L2 5

#define X_THETA 0
#define X_THETA_DOT 1
#define X_PSI_1_DOT 2
#define X_PSI_2_DOT 3
#define X_BIAS 4

typedef struct {
    RobotBase base;
    OLEDConfig oled_config;
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

    robot->params[A1] = -200;
    robot->params[A2] = -100;
    robot->params[A3] = -20;
    robot->params[B4] = 4;
    robot->params[L1] = 4;
    robot->params[L2] = -4;

    interface_param_init(&robot->oled_config, robot->params[A1]);
}

const char *param_names[PARAM_COUNT] = {
    "A1", "A2", "A3", "B4", "L11", "L51"
};

void robot_loop_active(RobotHandle robot_handle)
{
    Robot *robot = robot_handle;

    robot->x[X_THETA_DOT] = robot->base.y[Y_THETA_DOT];
    robot->x[X_PSI_1_DOT] =
        (0.5*R*ETA_1)*robot->base.y[2] + (0.5*R*ETA_2)*robot->base.y[3];
    robot->x[X_PSI_2_DOT] =
        (0.5*R_D_ratio*ETA_1)*robot->base.y[2] - (0.5*R_D_ratio*ETA_2)*robot->base.y[Y_PSI_2_DOT];

    robot->x[X_THETA] = robot->base.y[Y_THETA];// - robot->x[X_BIAS];

    robot->x[X_THETA_DOT] += (
        LAMBDA_2 * robot->x[X_THETA]
        - 0.5 * (ETA_1 * robot->base.u[U_1] + ETA_2 * robot->base.u[U_2])
        + robot->params[L1] * (robot->base.y[Y_THETA_DOT] - robot->x[X_THETA_DOT])
    ) * robot->base.dt;
    
    robot->x[X_BIAS] -= robot->params[L2] * (robot->base.y[Y_THETA_DOT] - robot->x[X_THETA_DOT]);

    robot->base.u[U_1] = - (
        robot->x[X_THETA] * robot->params[A1]*ETA_1_inv +
        robot->base.y[Y_THETA_DOT] * robot->params[A2]*ETA_1_inv +
        robot->x[X_PSI_1_DOT] * robot->params[A3]*ETA_1_inv +
        robot->x[X_PSI_2_DOT] * robot->params[B4]*ETA_1_inv
    );

    robot->base.u[U_2] = - (
        robot->x[X_THETA] * robot->params[A1]*ETA_2_inv +
        robot->base.y[Y_THETA_DOT] * robot->params[A2]*ETA_2_inv +
        robot->x[X_PSI_1_DOT] * robot->params[A3]*ETA_2_inv -
        robot->x[X_PSI_2_DOT] * robot->params[B4]*ETA_2_inv
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
        50
    );
}
