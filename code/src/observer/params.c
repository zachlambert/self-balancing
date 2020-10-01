#include "robot.h"
#include "constants.h"
#include "interface_param.h"
#include "zarduino/module/oled.h"
#include <stdlib.h>
#include <string.h>

#define PARAM_COUNT 5
#define A1 0
#define A2 1
#define A3 2
#define B4 3
#define L52 4

#define X_THETA 0
#define X_THETA_DOT 1
#define X_V 2
#define X_OMEGA 3
#define X_BIAS 4

#define MAX_BIAS 0.3

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

    robot->params[A1] = -600;
    robot->params[A2] = -150;
    robot->params[A3] = -30;
    robot->params[B4] = 0;
    robot->params[L52] = -1.55;

    interface_param_init(&robot->oled_config, robot->params[A1]);
}

const char *param_names[PARAM_COUNT] = {
    "A1", "A2", "A3", "B4", "L52"
};

void robot_loop_active(RobotHandle robot_handle)
{
    Robot *robot = robot_handle;

    robot->x[X_V] = R*robot->base.y[Y_THETA_DOT] +
        (0.5*R*ETA_1)*robot->base.y[Y_PSI_1_DOT] + (0.5*R*ETA_2)*robot->base.y[Y_PSI_2_DOT];
    robot->x[X_OMEGA] =
        (0.5*R_D_ratio*ETA_1)*robot->base.y[Y_PSI_1_DOT] - (0.5*R_D_ratio*ETA_2)*robot->base.y[Y_PSI_2_DOT];

    // X_THETA_DOT = Expected X_THETA_DOT at this timestep
    // Actual X_THETA - expected X_THETA
    // = (Current Y_THETA - Prev Y_THETA) - (expected change in Y_THETA)
    // Increase X_BIAS by (actual - expected) change in Y_THETA * L52 * dt
    robot->x[X_BIAS] += robot->params[L52] * (
        robot->base.y[Y_THETA_DOT] - robot->x[X_THETA_DOT]
    ) * robot->base.dt;
    if (robot->x[X_BIAS] > MAX_BIAS)
        robot->x[X_BIAS] = MAX_BIAS;
    else if (robot->x[X_BIAS] < -MAX_BIAS)
        robot->x[X_BIAS] = -MAX_BIAS;

    // Update X_THETA estimate from current Y_THETA and bias estimate to
    // satisfy Y_THETA = X_THETA + X_BIAS
    robot->x[X_THETA] = robot->base.y[Y_THETA] - robot->x[X_BIAS];

    robot->base.u[U_1] = - (
        robot->x[X_THETA] * robot->params[A1]*ETA_1_inv +
        robot->base.y[Y_THETA_DOT] * robot->params[A2]*ETA_1_inv +
        robot->x[X_V] * robot->params[A3]*ETA_1_inv +
        robot->x[X_OMEGA] * robot->params[B4]*ETA_1_inv
    );

    robot->base.u[U_2] = - (
        robot->x[X_THETA] * robot->params[A1]*ETA_2_inv +
        robot->base.y[Y_THETA_DOT] * robot->params[A2]*ETA_2_inv +
        robot->x[X_V] * robot->params[A3]*ETA_2_inv -
        robot->x[X_OMEGA] * robot->params[B4]*ETA_2_inv
    );

    // Expected X_THETA_DOT at next timestep = current Y_THETA_DOT
    // + dt * expected second derivative of theta
    robot->x[X_THETA_DOT] = robot->base.y[Y_THETA_DOT] + (
        LAMBDA_2*robot->x[X_THETA]
        - 0.5*OMEGA_2*(ETA_1*robot->base.u[U_1] + ETA_2*robot->base.u[U_2])
    ) * robot->base.dt;
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
    robot->x[X_BIAS] = 0;
}
