#include "motor.h"
#include <iostream>

Motor::Motor() = default;
Motor::~Motor()= default;

int Motor::start() {
    std::cout << "Motor started" << std::endl;
    return 0;
}

int Motor::stop() {
    std::cout << "Motor stopped" << std::endl;
    return 0;
}

void Motor::moveForward(int time) {
    std::cout << "Moving forward for " << time << " seconds" << std::endl;
}

void Motor::moveBackward(int time) {
    std::cout << "Moving backward for " << time << " seconds" << std::endl;
}

void Motor::moveRight() {
    std::cout << "Turning right" << std::endl;
}

void Motor::moveLeft() {
    std::cout << "Turning left" << std::endl;
}
