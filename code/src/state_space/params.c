#include "robot.h"
#include "constants.h"
#include "interface_param.h"
#include "zarduino/module/oled.h"
#include <stdlib.h>
#include <string.h>

#define A1 0
#define A2 1
#define A3 2
#define B4 3
#define V_CMD 4
#define OMEGA_CMD 5
#define PARAM_COUNT 6

#define X_THETA 0
#define X_THETA_DOT 1
#define X_VEL 2
#define X_OMEGA 3
#define STATE_COUNT 4

typedef struct {
    RobotBase base;
    OLEDConfig oled_config;
    float params[PARAM_COUNT];
    float x[STATE_COUNT];
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
    robot->params[V_CMD] = 0;
    robot->params[OMEGA_CMD] = 0;

    interface_param_init(&robot->oled_config, robot->params[0]);
}

const char *param_names[PARAM_COUNT] = {
    "A1", "A2", "A3", "B4", "V_CMD", "OM_CMD"
};

void robot_loop_active(RobotHandle robot_handle)
{
    Robot *robot = robot_handle;

    robot->x[X_THETA] = robot->base.y[Y_THETA];
    robot->x[X_THETA_DOT] = robot->base.y[Y_THETA_DOT];
    robot->x[X_VEL] = get_velocity(robot->base.y) - robot->params[V_CMD];
    robot->x[X_OMEGA] = robot->base.y[Y_PHI_DOT] - robot->params[OMEGA_CMD];

    robot->base.u[U_1] = - (
        robot->x[X_THETA] * robot->params[A1]*ETA_1_inv +
        robot->x[X_THETA_DOT] * robot->params[A2]*ETA_1_inv +
        robot->x[X_VEL] * robot->params[A3]*ETA_1_inv +
        robot->x[X_OMEGA] * robot->params[B4]*ETA_1_inv
    );

    robot->base.u[U_2] = - (
        robot->x[X_THETA] * robot->params[A1]*ETA_2_inv +
        robot->x[X_THETA_DOT] * robot->params[A2]*ETA_2_inv +
        robot->x[X_VEL] * robot->params[A3]*ETA_2_inv -
        robot->x[X_OMEGA] * robot->params[B4]*ETA_2_inv
    );
}

void robot_loop_inactive(RobotHandle robot_handle)
{
    Robot *robot = robot_handle;
    interface_param_update(
        &robot->oled_config,
        robot->params,
        param_names,
        PARAM_COUNT,
        50
    );
}
