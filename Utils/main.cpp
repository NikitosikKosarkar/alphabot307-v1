#include "motor.h"

int main() {
    Motor robot_motor;
    robot_motor.start();
    robot_motor.moveForward(5);
    robot_motor.moveRight();
    robot_motor.moveBackward(3);
    robot_motor.moveLeft();
    robot_motor.stop();
    return 0;
}