#include "Robot.h"
#include <iostream>

void Robot::prepare(Drink &drink) {
    std::cout << "Robot prepares " << drink.getName() << std::endl;
}

int Robot::start() {
    std::cout << "Robot started." << std::endl;
    return 0;
}

int Robot::stop() {
    std::cout << "Robot stopped." << std::endl;
    return 0;
}

void Robot::moveForward(int time) {
    std::cout << "Moving forward for " << time << std::endl;
}

void Robot::moveBackward(int time) {
    std::cout << "Moving backward for " << time << std::endl;
}

void Robot::turnLeft() {
    std::cout << "Robot turned left" << std::endl;
}

void Robot::turnRight() {
    std::cout << "Robot turned right" << std::endl;
}