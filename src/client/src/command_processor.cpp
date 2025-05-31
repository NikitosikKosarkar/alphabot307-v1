#include "command_processor.h"
#include "motion_controller.h"
#include "network_handler.h"

namespace RoboComm {

CommandProcessor::CommandProcessor(MotionController& ctrl, NetworkHandler& net)
    : controller_(ctrl), network_handler_(net), vision_mode_(false),
      auto_stop_timer_(0), movement_duration_(200) {}

void CommandProcessor::run() {
    cout << "CommandProcessor started. Awaiting commands..." << endl;
    running_ = true;

    while (running_) {
        if (!network_handler_.isConnected()) {
            cout << "MQTT disconnected. Waiting for reconnection..." << endl;
            sleep_for(milliseconds(1000));
            continue;
        }

        string cmd = network_handler_.getCommand();

        if (cmd.empty()) {
            if (vision_mode_ && auto_stop_timer_ > 0) {
                auto_stop_timer_--;
                if (auto_stop_timer_ <= 0) {
                    controller_.halt();
                    sendStatus("Auto-stopped after movement timeout");
                }
            }
            sleep_for(milliseconds(COMMAND_POLL_INTERVAL_MS));
            continue;
        }

        cout << "Command received: " << cmd << endl;
        processCommand(cmd);
    }

    cout << "CommandProcessor stopped." << endl;
}

void CommandProcessor::processCommand(const string& cmd) {
    try {
        if (cmd == "forward") {
            controller_.moveForward(getMovementDuration());
            setAutoStopTimer();
            sendStatus("Moving forward");
        }
        else if (cmd == "backward") {
            controller_.moveBackward(getMovementDuration());
            setAutoStopTimer();
            sendStatus("Moving backward");
        }
        else if (cmd == "left") {
            controller_.turnLeft(100);
            setAutoStopTimer();
            sendStatus("Turning left");
        }
        else if (cmd == "right") {
            controller_.turnRight(100);
            setAutoStopTimer();
            sendStatus("Turning right");
        }
        else if (cmd == "stop") {
            controller_.halt();
            auto_stop_timer_ = 0;
            sendStatus("Stopped");
        }

        else if (cmd == "start_vision") {
            vision_mode_ = true;
            movement_duration_ = 300;
            sendStatus("Vision mode activated - autonomous control enabled");
            cout << "Switched to autonomous vision mode" << endl;
        }
        else if (cmd == "stop_vision") {
            vision_mode_ = false;
            movement_duration_ = 200;
            controller_.halt();
            auto_stop_timer_ = 0;
            sendStatus("Manual mode activated - vision control disabled");
            cout << "Switched to manual control mode" << endl;
        }

        else if (cmd == "shutdown" || cmd == "shutdown_client") {
            cout << "Shutdown command received. Initiating graceful shutdown..." << endl;
            controller_.halt();
            sendStatus("Robot shutting down...");
            sleep_for(milliseconds(500));
            stop();
        }

        else if (cmd == "status") {
            string mode = vision_mode_ ? "AUTONOMOUS" : "MANUAL";
            string connection = network_handler_.isConnected() ? "CONNECTED" : "DISCONNECTED";
            sendStatus("Mode: " + mode + ", MQTT: " + connection + ", Robot: OPERATIONAL");
        }
        else if (cmd == "ping") {
            sendStatus("Robot client alive and responding");
        }

        else if (cmd == "fast") {
            movement_duration_ = 150;
            sendStatus("Movement speed set to FAST");
        }
        else if (cmd == "slow") {
            movement_duration_ = 400;

            sendStatus("Movement speed set to SLOW");
        }
        else if (cmd == "normal") {
            movement_duration_ = 200;
            sendStatus("Movement speed set to NORMAL");
        }

        else {
            cerr << "Unknown command: " << cmd << endl;
            sendStatus("ERROR: Unknown command '" + cmd + "'");
        }

    } catch (const std::exception& e) {
        cerr << "Error processing command '" << cmd << "': " << e.what() << endl;
        sendStatus("ERROR: Command processing failed - " + string(e.what()));
    }
}

void CommandProcessor::sendStatus(const string& message) {
    try {
        network_handler_.publishMessage("robot/status", message, 1, false);
        cout << "Status sent: " << message << endl;
    } catch (const std::exception& e) {
        cerr << "Failed to send status: " << e.what() << endl;
    }
}

int CommandProcessor::getMovementDuration() const {
    return movement_duration_;
}

void CommandProcessor::setAutoStopTimer() {
    if (vision_mode_) {
        auto_stop_timer_ = 20;
    }
}

void CommandProcessor::stop() {
    running_ = false;
    vision_mode_ = false;
    auto_stop_timer_ = 0;
}

}
