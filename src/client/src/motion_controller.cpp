#include "motion_controller.h"

namespace RoboComm {

void MotionController::activateMotors() {
    setPinState(6, 1);
    setPinState(26, 1);
}

void MotionController::setMovementPins(int in1, int in2, int in3, int in4) {
    setPinState(12, in1);
    setPinState(13, in2);
    setPinState(20, in3);
    setPinState(21, in4);
}

void MotionController::delay(int ms) {
    sleep_for(milliseconds(ms));
}

void MotionController::moveForward(int duration_ms) {
    activateMotors();
    setMovementPins(0, 1, 0, 1);
    delay(duration_ms);
    halt();
}

void MotionController::moveBackward(int duration_ms) {
    activateMotors();
    setMovementPins(1, 0, 1, 0);
    delay(duration_ms);
    halt();
}

void MotionController::turnRight(int duration_ms) {
    activateMotors();
    setMovementPins(0, 1, 1, 0);
    delay(duration_ms);
    halt();
}

void MotionController::turnLeft(int duration_ms) {
    activateMotors();
    setMovementPins(1, 0, 0, 1);
    delay(duration_ms);
    halt();
}

}