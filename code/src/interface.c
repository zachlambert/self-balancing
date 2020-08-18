#include "interface.h"

#include "zarduino/module/oled.h"
#include <stdint.h>
#include <stdio.h>

void interface_init(Robot *robot)
{
    robot->oled_config = oled_create_config();
    oled_init(&robot->oled_config);
    robot->oled_data = oled_create_data(&robot->oled_config);
}

void interface_update(Robot *robot, float dt)
{
    static char lines[6][30];

    static float seconds;
    seconds += dt;
    snprintf(lines[0], 30, "SECONDS: %lu\n", (uint32_t)seconds);

    snprintf(
        lines[1], 30, "RIGHT RPM: %u\n",
        (uint16_t)(robot->motor_left_vel*9.549)
    );
    snprintf(
        lines[2], 30, "LEFT RPM:  %u\n",
        (uint16_t)(robot->motor_right_vel*9.549)
    );
    snprintf(
        lines[3], 30, "TILT: %d\n",
        (int16_t)(robot->mpu6050_data.tilt*57.3)
    );
    snprintf(
        lines[4], 30, "TILT VEL: %d\n",
        (int16_t)robot->mpu6050_data.gyro[0]
    );
    snprintf(
        lines[5], 30, "PAN VEL: %d\n",
        (int16_t)robot->mpu6050_data.gyro[2]
    );

    oled_clear(&robot->oled_data);
    for (size_t i = 0; i < 6; i++) {
        oled_print_string(
            &robot->oled_config,&robot->oled_data, lines[i]
        );
    }
    oled_update(&robot->oled_config, &robot->oled_data);
}
