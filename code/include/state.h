#ifndef STATE_H
#define STATE_H

typedef struct {
    float v_cmd, omega_cmd;
    float theta, theta_dot;
    float phi_dot;
    float psi_right_dot, psi_left_dot;
    float motor_cmd_right, motor_cmd_left;
    float seconds, dt;
} State;

#endif
