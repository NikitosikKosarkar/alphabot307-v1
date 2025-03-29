#ifndef ISERVER_H
#define ISERVER_H

#include "Order.h"

class IServer {
public:
    virtual void control(Order& order) = 0;
    virtual ~IServer() = default;
};

#endif // ISERVER_H
