#pragma once

#include "common.h"

namespace RoboComm {

class MotionController;
class NetworkHandler;

class CommandProcessor {
    MotionController& controller_;
    NetworkHandler& network_handler_;
    bool running_ = true;

public:
    CommandProcessor(MotionController& ctrl, NetworkHandler& net);
    ~CommandProcessor() = default;

    void run();
    void stop();
};

}