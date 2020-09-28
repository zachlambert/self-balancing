#include "robot.h"
#include "constants.h"
#include "interface_param.h"
#include "interface_uart.h"
#include "zarduino/module/oled.h"
#include <stdlib.h>
#include <string.h>

#define PARAM_COUNT 1
#define K 0

typedef struct {
    RobotBase base;
    OLEDConfig oled_config;
    float params[PARAM_COUNT];
} Robot;

RobotHandle robot_create(void)
{
    Robot *robot = malloc(sizeof(Robot));
    memset(robot, 0, sizeof(*robot));
    robot->params[0] = 500;
    return robot;
}

void robot_init(RobotHandle robot_handle)
{
    Robot *robot = robot_handle;
    interface_uart_init();
    robot->oled_config = oled_create_config();
    interface_param_init(&robot->oled_config, robot->params[0]);
}

const char *param_names[1] = {
    "K"
};

void robot_loop_active(RobotHandle robot_handle)
{
    Robot *robot = robot_handle;
    robot->base.u[U_1] =
        robot->params[K] * robot->base.y[Y_THETA] / ETA_1;
    robot->base.u[U_2] =
        robot->params[K] * robot->base.y[Y_THETA] / ETA_2;
    interface_uart_send_state(robot_handle);
}

void robot_loop_inactive(RobotHandle robot_handle)
{
    Robot *robot = robot_handle;
    interface_param_update(
        &robot->oled_config,
        robot->params,
        param_names,
        PARAM_COUNT,
        400
    );
}
