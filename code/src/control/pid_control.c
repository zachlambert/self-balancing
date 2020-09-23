#include "control.h"
#include <stdio.h>
#include <string.h>

typedef struct {
    float kp, ki, kd, kie_limit;
    float e, e_prev, e_deriv, kie_sum;
} Controller;

const size_t CONTROLLER_PARAM_COUNT = 5;

ControllerHandle controller_init(void)
{
    Controller *controller = malloc(sizeof(Controller));
    memset(controller, 0, sizeof(Controller));
    return controller;
}

void controller_update(
    State *state,
    ControllerHandle controller_handle)
{
    Controller *controller = controller_handle;

    controller->e_prev = controller->e;
    controller->e = 0 - state->theta;
    controller->e_deriv = -state->theta_dot;

    controller->kie_sum +=
        controller->ki * 0.5 * (controller->e_prev + controller->e) * state->dt;
    if (controller->kie_sum > controller->kie_limit)
        controller->kie_sum = controller->kie_limit;
    else if (controller->kie_sum < -controller->kie_limit)
        controller->kie_sum = -controller->kie_limit;

    state->motor_cmd_right =
        controller->kp * controller->e +
        controller->kie_sum +
        controller->kd * controller->e_deriv;

    state->motor_cmd_left = state->motor_cmd_right;
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
        case 4:
            return 0;
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
        case 4:
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
                line, LINE_BUF_SIZE,"KP: %i\n",
                (int16_t)(1000*controller->kp)
            );
            break;
        case 1:
            snprintf(
                line, LINE_BUF_SIZE,"KI: %i\n",
                (int16_t)(1000*controller->ki)
            );
            break;
        case 2:
            snprintf(
                line, LINE_BUF_SIZE,"KD: %i\n",
                (int16_t)(1000*controller->kd)
            );
            break;
        case 3:
            snprintf(
                line, LINE_BUF_SIZE,"KIE_LIM: %i\n",
                (int16_t)(1000*controller->kie_limit)
            );
            break;
        case 4:
            snprintf(
                line, LINE_BUF_SIZE,"KIE_SUM: %i\n",
                (int16_t)(1000*controller->kie_sum)
            );
            break;
        default:
            break;
    }
    return line;
}
