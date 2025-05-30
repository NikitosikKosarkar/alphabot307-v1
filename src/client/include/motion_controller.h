#pragma once

#include "pin_manager.h"

namespace RoboComm {

class MotionController : public PinManager {
public:
    MotionController() = default;
    ~MotionController() override = default;

    void moveForward(int duration_ms);
    void moveBackward(int duration_ms);
    void turnRight(int duration_ms);
    void turnLeft(int duration_ms);

private:
    void activateMotors();
    void setMovementPins(int in1, int in2, int in3, int in4);
    void delay(int ms);
};

}
