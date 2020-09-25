#include "robot.h"
#include "interface_uart.h"
#include <stdlib.h>
#include <string.h>

const float K = 100;

RobotHandle robot_create(void)
{
    RobotBase *robot = malloc(sizeof(RobotBase));
    memset(robot, 0, sizeof(*robot));
    return robot;
}

void robot_init(RobotHandle robot_handle)
{
    interface_uart_init();
}

void robot_loop(RobotHandle robot_handle)
{
    RobotBase *robot = robot_handle;
    if (robot->active) {
        robot->motor_cmd_left += K*robot->theta * robot->dt;
        robot->motor_cmd_right = robot->motor_cmd_left;
        interface_uart_send_state(robot_handle);
    }
}
