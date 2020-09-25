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

void robot_loop(RobotHandle robot_handle)
{
    if (((RobotBase*)robot_handle)->active) {
        interface_uart_send_state(robot_handle);
    }
}
