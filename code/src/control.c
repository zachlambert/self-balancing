#include "control.h"
#include "motors.h"

ControlState control_create(void)
{
    ControlState state;
    state.controller_type = CONTROLLER_TYPE_PID;
    state.controller_pid.kp = 2;
    state.controller_pid.ki = 0;
    state.controller_pid.kd = 0;
    state.controller_pid.kie_limit = 10;
    return state;
}

void control_update(ControlState *state, float dt)
{
    static float error, control;
    static const float theta_cmd = 0;

    error = theta_cmd - state->theta;
    control = state->controller_pid.kp * error;

    state->motor_left_input = control;
    state->motor_right_input = control;
}
