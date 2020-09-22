#ifndef CONTROL_H
#define CONTROL_H

#include <stdlib.h>

typedef struct {
    float v_cmd, omega_cmd;
    float theta, theta_dot;
    float phi_dot;
    float psi_right_dot, psi_left_dot;
    float motor_cmd_right, motor_cmd_left;
} State;

typedef void *ControllerHandle;
extern const size_t CONTROLLER_PARAM_COUNT;

ControllerHandle controller_init(void);
void controller_update(State *state, ControllerHandle controller_handle, float dt);
float controller_get_param(
    ControllerHandle conroller_handle, size_t controller_param_i
);
void controller_set_param(
    ControllerHandle conroller_handle, size_t controller_param_i, float value
);
char *controller_get_string(ControllerHandle controller_handle, size_t param_i);

#endif
