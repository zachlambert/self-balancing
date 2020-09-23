#include "robot.h"
#include "zarduino/timing/delay.h"

int main(void)
{
    delay(2000);
    static Robot robot; // Make static so it shows up in data usage
    robot_init(&robot);

    while (1) {
        robot_loop(&robot);
    }
}
