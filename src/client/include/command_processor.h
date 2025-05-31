#pragma once

#include "common.h"

namespace RoboComm {

class MotionController;
class NetworkHandler;

class CommandProcessor {
private:
    MotionController& controller_;
    NetworkHandler& network_handler_;

    bool running_ = false;
    bool vision_mode_ = false;
    int auto_stop_timer_ = 0;
    int movement_duration_ = 200;

    static const int COMMAND_POLL_INTERVAL_MS = 100;

    void processCommand(const string& cmd);
    void sendStatus(const string& message);
    int getMovementDuration() const;
    void setAutoStopTimer();

public:
    CommandProcessor(MotionController& ctrl, NetworkHandler& net);

    void run();
    void stop();

    bool isVisionMode() const { return vision_mode_; }
    bool isRunning() const { return running_; }
};

}
