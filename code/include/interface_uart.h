#ifndef INTERFACE_UART_H
#define INTERFACE_UART_H

#include "robot.h"

void interface_uart_init(void);
void interface_uart_send_state(RobotHandle robot_handle);

#endif
