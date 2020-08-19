#include "control.h"
#include "motors.h"

ControlState control_create(void)
{
    ControlState state;
    state.controller_type = CONTROLLER_TYPE_PID;
    state.controller_pid.kp = 0.1;
    state.controller_pid.ki = 0;
    state.controller_pid.kd = 0;
    state.controller_pid.kie_limit = 10;
    return state;
}

void control_update(ControlState *state, float dt)
{
    // todo
}
