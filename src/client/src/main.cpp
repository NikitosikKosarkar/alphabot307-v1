#include "common.h"
#include "pin_manager.h"
#include "motion_controller.h"
#include "network_handler.h"
#include "command_processor.h"

#include <csignal>
#include <atomic>

std::atomic<bool> g_shutdown_flag(false);

void signal_handler(int signum) {
    cerr << "Interrupt signal (" << signum << ") received." << endl;
    g_shutdown_flag.store(true);
}


int main(int argc, char* argv[]) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    const char* mqtt_broker_address = "192.168.1.102";
    if (argc > 1) {
        mqtt_broker_address = argv[1];
        cout << "Using MQTT broker address from command line: " << mqtt_broker_address << endl;
    } else {
        cout << "Using default MQTT broker address: " << mqtt_broker_address << endl;
    }

    const char* robot_id = "robot_client_01";
    const char* command_topic = "robot/gpio";


    try {
        RoboComm::MotionController motion_ctrl;
        RoboComm::NetworkHandler network_handler(robot_id, mqtt_broker_address, RoboComm::DEFAULT_MQTT_PORT, command_topic);
        RoboComm::CommandProcessor cmd_processor(motion_ctrl, network_handler);

        thread processor_thread([&cmd_processor]() {
            cmd_processor.run();
        });

        while(!g_shutdown_flag.load()) {
            if (!network_handler.isConnected()){

            }
            sleep_for(milliseconds(500));
        }

        cout << "Initiating shutdown..." << endl;
        cmd_processor.stop();
        if (processor_thread.joinable()) {
            processor_thread.join();
        }

        motion_ctrl.halt();

    } catch (const std::exception& e) {
        cerr << "Fatal error in main: " << e.what() << endl;
        return 1;
    } catch (...) {
        cerr << "Unknown fatal error in main." << endl;
        return 1;
    }

    cout << "Robot client shut down gracefully." << endl;
    return 0;
}
