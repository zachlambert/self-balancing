#include "robot.h"
#include "interface_param.h"
#include "zarduino/module/oled.h"
#include <stdlib.h>
#include <string.h>

#define PARAM_COUNT 6
#define KP 0
#define KI 1
#define KD 2
#define KIE_LIM 3
#define U_KP 4
#define U_KI 5

typedef struct {
    RobotBase base;
    OLEDConfig oled_config;
    float params[PARAM_COUNT];
    float e, e_prev, e_deriv, kie_sum;
    float eff, eff_prev;
    float u_e, u_e_prev, u_kie_sum;
} Robot;

RobotHandle robot_create(void)
{
    Robot *robot = malloc(sizeof(Robot));
    memset(robot, 0, sizeof(*robot));
    return robot;
}

void robot_init(RobotHandle robot_handle)
{
    Robot *robot = robot_handle;
    robot->oled_config = oled_create_config();
    interface_param_init(&robot->oled_config, 0);
}

const char *param_names[6] = {
    "KP", "KI", "KD", "KIE_LIM", "U_KP", "U_KI"
};

void robot_loop(RobotHandle robot_handle)
{
    Robot *robot = robot_handle;

    if (robot->base.active) {
        robot->e_prev = robot->e;
        robot->e = 0 - robot->base.theta;
        robot->e_deriv = -robot->base.theta_dot;

        robot->kie_sum +=
            robot->params[KI] * 0.5 * (robot->e_prev + robot->e) * robot->base.dt;
        if (robot->kie_sum > robot->params[KIE_LIM])
            robot->kie_sum = robot->params[KIE_LIM];
        else if (robot->kie_sum < -robot->params[KIE_LIM])
            robot->kie_sum = -robot->params[KIE_LIM];

        robot->u_e_prev = robot->u_e;
        robot->u_e = 0 - robot->base.psi_left_dot;
        robot->u_kie_sum +=
            robot->params[U_KI] * 0.5 * (robot->u_e_prev + robot->u_e) * robot->base.dt;

        robot->eff_prev = robot->eff;
        robot->eff =
            robot->params[KP] * robot->e +
            robot->kie_sum +
            robot->params[KD] * robot->e_deriv +
            robot->params[U_KP] * robot->u_e +
            robot->u_kie_sum;

        robot->base.motor_cmd_right +=
            0.5 * (robot->eff_prev + robot->eff) * robot->base.dt;
        robot->base.motor_cmd_left = robot->base.motor_cmd_right;

    } else {
        interface_param_update(
            &robot->oled_config,
            robot->params,
            param_names,
            PARAM_COUNT,
            2
        );
    }
}
