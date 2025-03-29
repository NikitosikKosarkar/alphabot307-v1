#include "Robot.h"
#include <iostream>
#include <stdexcept>

int Robot::start() {
    std::cout << "Robot: Movement system started" << std::endl;
    return 0;
}

int Robot::stop() {
    std::cout << "Robot: Movement system stopped" << std::endl;
    return 0;
}

void Robot::moveForward(int time) {
    if (time < 0) throw std::invalid_argument("Time cannot be negative");
    std::cout << "Robot: Moving forward for " << time << " seconds" << std::endl;
}

void Robot::moveBackward(int time) {
    if (time < 0) throw std::invalid_argument("Time cannot be negative");
    std::cout << "Robot: Moving backward for " << time << " seconds" << std::endl;
}

void Robot::turnLeft() {
    std::cout << "Robot: Turning left" << std::endl;
}

void Robot::turnRight() {
    std::cout << "Robot: Turning right" << std::endl;
}

void Robot::prepare(Drink& drink) {
    std::cout << "Robot: Preparing " << drink.getName() << std::endl;
}
