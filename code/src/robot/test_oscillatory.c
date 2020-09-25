#include "robot.h"
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
    robot->params[0] = 200;
    return robot;
}

void robot_init(RobotHandle robot_handle)
{
    Robot *robot = robot_handle;
    interface_uart_init();
    robot->oled_config = oled_create_config();
    interface_param_init(&robot->oled_config);
}

const char *param_names[1] = {
    "K"
};

void robot_loop(RobotHandle robot_handle)
{
    Robot *robot = robot_handle;
    if (robot->base.active) {
        robot->base.motor_cmd_left +=
            robot->params[K] * robot->base.theta * robot->base.dt;
        robot->base.motor_cmd_right = robot->base.motor_cmd_left;
        interface_uart_send_state(robot_handle);
    } else {
        interface_param_update(
            &robot->oled_config,
            robot->params,
            param_names,
            PARAM_COUNT
        );
    }
}
