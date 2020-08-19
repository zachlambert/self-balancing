#ifndef CONTROL_H
#define CONTROL_H

typedef enum {
    CONTROLLER_TYPE_PID
} ControllerType;

typedef struct {
    float v_cmd; // Won't use these at the moment
    float omega_cmd;

    float theta;
    float theta_dot;
    float phi_dot;
    float psi_1_dot;
    float psi_2_dot;

    float motor_left_input;
    float motor_right_input;

    struct {
        float R, L, H;
        // Wheel radius, dist wheel->centre, centre of mass height
    } dimensions;

    ControllerType controller_type;
    union {
        struct {
            float kp, ki, kd, kie_limit;
            float e_prev;
            float kie_sum;
        } controller_pid;
    };
} ControlState;

ControlState control_create(void);
void control_update(ControlState *state, float dt);

#endif
