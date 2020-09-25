#include "control.h"
#include <stdio.h>
#include <string.h>

typedef struct {
    float kp, ki, kd, kie_limit;
    float e, e_prev, e_deriv, kie_sum;
    float eff, eff_prev;
    float u_kp, u_ki;
    float u_e, u_e_prev, u_kie_sum;
} Controller;

const size_t CONTROLLER_PARAM_COUNT = 6;

ControllerHandle controller_init(void)
{
    Controller *controller = malloc(sizeof(Controller));
    controller->kp = 0;
    controller->kd = 0;
    controller->kie_limit = 0;
    controller->e = 0;
    controller->kie_sum = 0;
    controller->u_kp = 0;
    controller->u_ki = 0;
    controller->u_kie_sum = 0;
    controller->u_e = 0;

    controller->ki = 0;
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

    controller->u_e_prev = controller->u_e;
    controller->u_e = 0 - state->psi_left_dot;
    controller->u_kie_sum +=
        controller->u_ki * 0.5 * (controller->u_e_prev + controller->u_e) * state->dt;

    controller->eff_prev = controller->eff;
    controller->eff =
        controller->kp * controller->e +
        controller->kie_sum +
        controller->kd * controller->e_deriv +
        controller->u_kp * controller->u_e +
        controller->u_kie_sum;

    state->motor_cmd_right += 0.5 * (controller->eff_prev + controller->eff) * state->dt;
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
            return controller->u_kp;
        case 5:
            return controller->u_ki;
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
            controller->u_kp = value;
            break;
        case 5:
            controller->u_ki = value;
            break;
        default:
            break;
    }
}

const char format_strings[6][8] = {
    "KP", "KI", "KD", "KIE_LIM", "U_KP", "U_KI"
};
const char *controller_get_param_name(size_t param_i)
{
    return format_strings[param_i];
}
