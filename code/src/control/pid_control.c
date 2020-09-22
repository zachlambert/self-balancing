#include "control.h"
#include <stdio.h>

typedef struct {
    float kp, ki, kd, kie_limit;
    float e, e_prev, e_deriv, kie_sum;
} Controller;

const size_t CONTROLLER_PARAM_COUNT = 4;

ControllerHandle controller_init(void)
{
    Controller *controller = malloc(sizeof(Controller));
    controller->kp = 10;
    controller->ki = 2.8;
    controller->kd = 0.1;
    controller->kie_limit = 20;
    controller->e_prev = 0;
    controller->kie_sum = 0;
    return controller;
}

void controller_update(
    State *state,
    ControllerHandle controller_handle,
    float dt)
{
    Controller *controller = controller_handle;

    controller->e = 0 - state->theta;
    controller->e_deriv = -state->theta_dot;

    controller->kie_sum +=
        controller->ki * 0.5 * (controller->e - controller->e_prev) * dt;
    if (controller->kie_sum > controller->kie_limit)
        controller->kie_sum = controller->kie_limit;
    else if (controller->kie_sum < -controller->kie_limit)
        controller->kie_sum = -controller->kie_limit;

    state->motor_cmd_1 =
        controller->kp * controller->e +
        controller->kie_sum +
        controller->kd * controller->e_deriv;

    state->motor_cmd_2 = state->motor_cmd_1;
}

float controller_get_param(
    ControllerHandle controller_handle,
    size_t param_i)
{
    Controller *controller = controller_handle;
    switch (param_i) {
        case 0:
            return controller->kp;
        case 1:
            return controller->ki;
        case 2:
            return controller->kd;
        case 3:
            return controller->kie_limit;
        default:
            return 0;
    }
}

void controller_set_param(
    ControllerHandle controller_handle,
    size_t param_i,
    float value)
{
    Controller *controller = controller_handle;
    switch (param_i) {
        case 0:
            controller->kp = value;
            break;
        case 1:
            controller->ki = value;
            break;
        case 2:
            controller->kd = value;
            break;
        case 3:
            controller->kie_limit = value;
            break;
        default:
            break;
    }
}


#define LINE_BUF_SIZE 50
char line[LINE_BUF_SIZE];

char *controller_get_string(ControllerHandle controller_handle, size_t param_i)
{
    Controller *controller = controller_handle;
    switch (param_i) {
        case 0:
            snprintf(
                line, LINE_BUF_SIZE,"KP: %i",
                (int16_t)(1000*controller->kp)
            );
            break;
        case 1:
            snprintf(
                line, LINE_BUF_SIZE,"KI: %i",
                (int16_t)(1000*controller->ki)
            );
            break;
        case 2:
            snprintf(
                line, LINE_BUF_SIZE,"KD: %i",
                (int16_t)(1000*controller->kd)
            );
            break;
        case 3:
            snprintf(
                line, LINE_BUF_SIZE,"KIE_LIM: %i",
                (int16_t)(1000*controller->kie_limit)
            );
            break;
        default:
            break;
    }
    return line;
}
