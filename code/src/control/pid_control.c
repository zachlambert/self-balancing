#include "control.h"
#include <stdlib.h>

typedef struct {
    float kp, ki, kd, kie_limit;
    float e, e_prev, e_deriv, kie_sum;
} Controller;

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

void controller_update(State *state, ControllerHandle controller_handle, float dt)
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
