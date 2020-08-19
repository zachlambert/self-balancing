#include "control.h"
#include "motors.h"

void control_robot(Robot *robot, float dt)
{
    static float kp = 5;
    static float ki = 1;
    static float kd = 1;
    static float input, output, error, ki_sum, deriv;
    static float ki_limit = 10;

    input = robot->mpu6050_data.roll;
    error = 0 - input;
    deriv = error / dt;
    ki_sum += ki * error;
    if (ki_sum > ki_limit) ki_sum = ki_limit;
    else if (ki_sum < -ki_limit) ki_sum = -ki_limit;
    output = kp*error + ki_sum + kd*deriv;

    if (output < 0) {
        motors_set_left(robot, -output, 0);
        motors_set_right(robot, -output, 0);
    } else {
        motors_set_left(robot, output, 1);
        motors_set_right(robot, output, 1);
    }
}
