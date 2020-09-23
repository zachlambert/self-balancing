#include "control.h"
#include <stdio.h>

const size_t CONTROLLER_PARAM_COUNT = 1;

ControllerHandle controller_init(void)
{
    return 0;
}

void controller_update(
    State *state,
    ControllerHandle controller_handle)
{
    if (state->seconds > 0.5) {
        state->motor_cmd_right = 1;
        state->motor_cmd_left = 1;
    }
}

float controller_get_param(
    ControllerHandle controller_handle,
    size_t param_i)
{
    return 0;
}

void controller_set_param(
    ControllerHandle controller_handle,
    size_t param_i,
    float value)
{}


char *controller_get_string(ControllerHandle controller_handle, size_t param_i)
{
    return 0;
}
