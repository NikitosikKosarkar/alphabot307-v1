#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <stdexcept>
#include <sstream>
#include <fstream>
#include <cstring>
#include <memory>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>

#include <mosquitto.h>

using std::string;
using std::vector;
using std::cout;
using std::cerr;
using std::endl;
using std::mutex;
using std::lock_guard;
using std::thread;
using std::runtime_error;
using std::exception;
using std::chrono::milliseconds;
using std::this_thread::sleep_for;
using std::unique_ptr;
using std::make_unique;
using std::shared_ptr;
using std::make_shared;

namespace RoboComm {

constexpr int DEFAULT_MQTT_PORT = 1883;
constexpr int GPIO_CHIP_TIMEOUT = 60;
constexpr int COMMAND_POLL_INTERVAL_MS = 100;
constexpr int MQTT_KEEP_ALIVE_S = 60;
constexpr int MQTT_LOOP_TIMEOUT_MS = 100;


class NonCopyable {
protected:
    NonCopyable() = default;
    ~NonCopyable() = default;
    NonCopyable(const NonCopyable&) = delete;
    NonCopyable& operator=(const NonCopyable&) = delete;
};

}