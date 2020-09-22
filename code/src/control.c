#include "control.h"
#include "motors.h"

ControlState control_create(void)
{
    ControlState state;
    state.controller_type = CONTROLLER_TYPE_PID;
    state.controller_pid.kp = 10;
    state.controller_pid.ki = 2.8;
    state.controller_pid.kd = 0.1;
    state.controller_pid.kie_limit = 20;
    state.controller_pid.e_prev = 0;
    state.controller_pid.kie_sum = 0;
    return state;
}

void control_update(ControlState *state, float dt)
{
    static float e, e_deriv, control;
    static const float theta_cmd = 0;

    e = theta_cmd - state->theta;
    e_deriv = -state->theta_dot;

    state->controller_pid.kie_sum +=
        state->controller_pid.ki * 0.5 * (e - state->controller_pid.e_prev) * dt;
    if (state->controller_pid.kie_sum > state->controller_pid.kie_limit)
        state->controller_pid.kie_sum = state->controller_pid.kie_limit;
    else if (state->controller_pid.kie_sum < -state->controller_pid.kie_limit)
        state->controller_pid.kie_sum = -state->controller_pid.kie_limit;

    control =
        state->controller_pid.kp * e +
        state->controller_pid.kie_sum +
        state->controller_pid.kd * e_deriv;

    // Need to also take psi into account, to try and stabilise with
    // wheels stationary. Need to add pid parameters for this error too.
    // TODO

    state->motor_left_input = control;
    state->motor_right_input = control;
}
