#include "interface_param.h"

#include "zarduino/comms/uart.h"

#define INTERFACE_LINE_SIZE 16

void interface_uart_init(void)
{
    UartConfig uart_config = uart_create_config();
    uart_config.baud_rate = 57600;
    uart_init(&uart_config);
}

void interface_send_state(RobotHandle robot_handle)
{
    RobotBase *robot = robot_handle;
    uart_write_int32(robot->seconds * 10000);
    uart_write_int32(robot->theta * 10000);
    uart_write_int32(robot->theta_dot * 10000);
    uart_write_int32(robot->phi_dot * 10000);
    uart_write_int32(robot->psi_right_dot * 10000);
    uart_write_int32(robot->psi_left_dot * 10000);
    uart_write_int32(robot->motor_cmd_right * 10000);
    uart_write_int32(robot->motor_cmd_left * 10000);
}
