#include "interface_param.h"

#include "zarduino/comms/uart.h"

#define INTERFACE_LINE_SIZE 16

void interface_uart_init(void)
{
    UartConfig uart_config = uart_create_config();
    uart_config.baud_rate = 57600;
    uart_init(&uart_config);
}

void interface_uart_send_state(RobotHandle robot_handle)
{
    RobotBase *robot = robot_handle;
    uart_write_int32(robot->seconds * 10000);
    for (size_t i = 0; i < 2; i++) {
        uart_write_int32(robot->pwm[i] * 10000);
    };
    for (size_t i = 0; i < 2; i++) {
        uart_write_int32(robot->u[i] * 10000);
    };
    for (size_t i = 0; i < 4; i++) {
        uart_write_int32(robot->y[i] * 10000);
    }
}
