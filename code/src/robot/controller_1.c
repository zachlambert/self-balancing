#include "robot.h"
#include "constants.h"
#include "interface_param.h"
#include "zarduino/module/oled.h"
#include <stdlib.h>
#include <string.h>

#define PARAM_COUNT 8

typedef struct {
    RobotBase base;
    OLEDConfig oled_config;
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
    robot->oled_config = oled_create_config();

    robot->params[0] = -25;
    robot->params[1] = -7;
    robot->params[2] = -4;
    robot->params[3] = 2;
    robot->params[4] = -25;
    robot->params[5] = -7;
    robot->params[6] = -4;
    robot->params[7] = 2;

    interface_param_init(&robot->oled_config, robot->params[0]);
}

const char *param_names[PARAM_COUNT] = {
    "K11", "K12", "K13", "K14", "K21", "K22", "K23", "K24"
};

void robot_loop_active(RobotHandle robot_handle)
{
    Robot *robot = robot_handle;

    robot->x[0] = robot->base.y[0];
    robot->x[1] = robot->base.y[1];
    robot->x[2] = 0.5*R*ETA_1*robot->base.y[2] + 0.5*R*ETA_2*robot->base.y[3];
    robot->x[3] = 0.5*R*ETA_1*robot->base.y[2]/D - 0.5*R*ETA_2*robot->base.y[3]/D;

    robot->base.u[U_1] = 0;
    robot->base.u[U_2] = 0;
    for (size_t i = 0; i < 4; i++) {
        robot->base.u[U_1] -= robot->params[i] * robot->x[i];
        robot->base.u[U_2] -= robot->params[4+i] * robot->x[i];
    }
}

void robot_loop_inactive(RobotHandle robot_handle)
{
    Robot *robot = robot_handle;
    interface_param_update(
        &robot->oled_config,
        robot->params,
        param_names,
        PARAM_COUNT,
        5
    );
}
