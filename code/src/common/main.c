#include "robot.h"
#include "motors.h"
#include "interface_common.h"

#include "zarduino/timing/delay.h"
#include "zarduino/timing/timing.h"
#include "zarduino/comms/i2c.h"

void common_init(RobotHandle robot_handle)
{
    RobotBase *robot = robot_handle;

    motors_init();

    I2CConfig i2c_config = i2c_create_config();
    i2c_init_master(&i2c_config);

    robot->mpu6050_config = mpu6050_create_config();
    mpu6050_init(&robot->mpu6050_config);

    timer0_init_as_timer_accurate();

    interface_common_init();
}

uint64_t current_ticks = 0;
uint64_t prev_ticks, delta_ticks;
inline void read_time(RobotHandle robot_handle)
{
    RobotBase *robot = robot_handle;
    prev_ticks = current_ticks;
    current_ticks = timer0_accurate_get_ticks();
    delta_ticks = current_ticks - prev_ticks;
    robot->dt = ((float)delta_ticks) * 64e-9;
    robot->seconds += robot->dt;
}

inline void read_sensors(RobotHandle robot_handle)
{
    RobotBase *robot = robot_handle;

    motors_get_feedback(&robot->y[Y_PSI_1_DOT], &robot->y[Y_PSI_2_DOT], robot->dt);
    mpu6050_read_data(&robot->mpu6050_config, &robot->mpu6050_data);

    robot->y[Y_THETA] = atan2(
        -robot->mpu6050_data.accel[0], robot->mpu6050_data.accel[2]
    );
    robot->y[Y_THETA_DOT] = robot->mpu6050_data.gyro[1] * 0.01745;

    robot->y[Y_PHI_DOT] =
        cos(robot->y[Y_THETA]) * robot->mpu6050_data.gyro[2] * 0.01745
        - sin(robot->y[Y_THETA]) * robot->mpu6050_data.gyro[0] * 0.01745;
}

inline void update_state(RobotHandle robot_handle)
{
    RobotBase *robot = robot_handle;
    if (robot->active) {
        robot_loop_active(robot_handle);
        robot->pwm[PWM_1] += robot->u[U_1] * robot->dt;
        robot->pwm[PWM_2] += robot->u[U_2] * robot->dt;
    } else {
        robot_loop_inactive(robot_handle);
    }
}

inline void write_motors(RobotHandle robot_handle)
{
    RobotBase *robot = robot_handle;
    motors_set_pwm_1(&robot->pwm[PWM_1]);
    motors_set_pwm_2(&robot->pwm[PWM_2]);
    // The above function will threshold pwm if magnitude > 1
}

int main(void)
{
    delay(2000);
    RobotHandle robot_handle = robot_create();
    common_init(robot_handle);
    robot_init(robot_handle);

    while (1) {
        read_time(robot_handle);
        read_sensors(robot_handle);
        update_state(robot_handle);
        write_motors(robot_handle);
        interface_common_update(robot_handle);
    }
}
