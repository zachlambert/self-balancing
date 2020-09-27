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
} Robot;

RobotHandle robot_create(void)
{
    Robot *robot = malloc(sizeof(Robot));
    memset(robot, 0, sizeof(*robot));
    return robot;
}

void robot_init(RobotHandle robot_handle)
{
    interface_uart_init();
}

void robot_loop_active(RobotHandle robot_handle)
{
    Robot *robot = robot_handle;
    robot->base.u[U_1] = 1;
    robot->base.u[U_2] = 1;
    interface_uart_send_state(robot_handle);
}

void robot_loop_inactive(RobotHandle robot_handle)
{
    // Do nothing
}
