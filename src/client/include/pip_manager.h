#pragma once

#include "common.h"

namespace RoboComm {

class PinManager : private NonCopyable {
protected:
    vector<int> pin_numbers_ = {12, 13, 6, 20, 21, 26};
    int device_handler_ = -1;

    void setPinState(int pin, uint8_t state);

public:
    PinManager();
    virtual ~PinManager();

    void halt();
};

}