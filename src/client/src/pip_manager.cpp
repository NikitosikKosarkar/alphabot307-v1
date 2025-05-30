#include "pin_manager.h"
#include <system_error>

namespace RoboComm {

PinManager::PinManager() {
    device_handler_ = open("/dev/gpiochip0", O_RDWR | O_CLOEXEC);
    if (device_handler_ < 0) {
        throw std::system_error(errno, std::generic_category(), "GPIO init failure: Failed to open /dev/gpiochip0");
    }

    for (int p : pin_numbers_) {
        setPinState(p, 0);
    }
    halt();
}

PinManager::~PinManager() {
    if (device_handler_ >= 0) {
        try {
            halt();
        } catch (const std::exception& e) {
            cerr << "Error during halt in PinManager destructor: " << e.what() << endl;
        }
        close(device_handler_);
    }
}

void PinManager::setPinState(int pin, uint8_t state) {
    if (device_handler_ < 0) {
        throw runtime_error("GPIO device not open.");
    }

    struct gpiohandle_request config_req;
    struct gpiohandle_data pin_data;
    memset(&config_req, 0, sizeof(config_req));
    memset(&pin_data, 0, sizeof(pin_data));

    strncpy(config_req.consumer_label, "robo_driver", sizeof(config_req.consumer_label) - 1);
    config_req.lineoffsets[0] = pin;
    config_req.lines = 1;
    config_req.flags = GPIOHANDLE_REQUEST_OUTPUT;

    int ret = ioctl(device_handler_, GPIO_GET_LINEHANDLE_IOCTL, &config_req);
    if (ret < 0) {
        throw std::system_error(errno, std::generic_category(), "Pin access error for pin " + std::to_string(pin));
    }

    pin_data.values[0] = state;
    ret = ioctl(config_req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &pin_data);
    if (ret < 0) {
        close(config_req.fd);
        throw std::system_error(errno, std::generic_category(), "State change error for pin " + std::to_string(pin));
    }

    close(config_req.fd);
}

void PinManager::halt() {
    setPinState(6, 1);
    setPinState(26, 1);
    setPinState(12, 0);
    setPinState(13, 0);
    setPinState(20, 0);
    setPinState(21, 0);
}

}