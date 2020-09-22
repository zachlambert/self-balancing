#ifndef CONTROL_H
#define CONTROL_H

typedef struct {
    float v_cmd, omega_cmd;
    float theta, theta_dot;
    float phi_dot;
    float psi_1_dot, psi_2_dot;
    float motor_cmd_1, motor_cmd_2;
} State;

typedef void *ControllerHandle;
ControllerHandle controller_init(void);
void controller_update(State *state, ControllerHandle controller_handle, float dt);
// void controller_update_interface(Robot *robot);

#endif
