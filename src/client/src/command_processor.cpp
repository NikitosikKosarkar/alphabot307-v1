#include "command_processor.h"
#include "motion_controller.h"
#include "network_handler.h"

namespace RoboComm {

CommandProcessor::CommandProcessor(MotionController& ctrl, NetworkHandler& net)
    : controller_(ctrl), network_handler_(net) {}

void CommandProcessor::run() {
    cout << "CommandProcessor started. Awaiting commands..." << endl;
    running_ = true;
    while (running_) {
        if (!network_handler_.isConnected()) {

            sleep_for(milliseconds(1000));
            continue;
        }

        string cmd = network_handler_.getCommand();

        if (cmd.empty()) {
            sleep_for(milliseconds(COMMAND_POLL_INTERVAL_MS));
            continue;
        }

        cout << "Command received: " << cmd << endl;

        if (cmd == "forward") controller_.moveForward(200);
        else if (cmd == "backward") controller_.moveBackward(200);
        else if (cmd == "stop") controller_.halt();
        else if (cmd == "left") controller_.turnLeft(100);
        else if (cmd == "right") controller_.turnRight(100);
        else if (cmd == "shutdown_client") {
            cout << "Shutdown command received. Exiting." << endl;
            stop();
        }
        else {
            cerr << "Invalid command: " << cmd << endl;
            network_handler_.publishMessage("robot/status", "Invalid command: " + cmd);
        }
    }
    cout << "CommandProcessor stopped." << endl;
}

void CommandProcessor::stop() {
    running_ = false;
}

}