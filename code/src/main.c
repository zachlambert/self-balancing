#include "robot.h"
#include "zarduino/timing/delay.h"

int main(void)
{
    delay(2000);
    Robot robot = robot_create();
    robot_init(&robot);

    while (1) {
        robot_loop(&robot);
    }
}
