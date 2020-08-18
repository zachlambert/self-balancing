#include "robot.h"

int main(void)
{
    Robot robot = robot_create();
    robot_init(&robot);

    motors_set_left(&robot, 0.3, 1);
    motors_set_right(&robot, 0.3, 1);

    mpu6050_calibrate(&robot.mpu6050_config);

    while (1) {
        robot_loop(&robot);
        // Writing to the oled creates enough of a delay
        // in the loop.
        // Using a delay function seems to break the timer,
        // so if I later remove the oled code from the loop,
        // use a timer callback to run the loop at regular
        // intervals, but don't rely on it being accurate, so
        // still use the accurate timer for measuring the
        // time elapsed.
    }
}
