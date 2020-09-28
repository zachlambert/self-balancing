#include "robot.h"
#include "interface_uart.h"
#include <stdlib.h>
#include <string.h>

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

void robot_loop_active(RobotHandle robot_handle)
{
    interface_uart_send_state(robot_handle);
}

void robot_loop_inactive(RobotHandle robot_handle)
{
    // Do nothing
}
