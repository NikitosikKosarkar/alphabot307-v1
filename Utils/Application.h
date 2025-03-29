#ifndef APPLICATION_H
#define APPLICATION_H
#include "Server.h"
#include "Order.h"

class Application {
    public:
        void sendOrder(Server& server, Order& order);
};

#endif //APPLICATION_H
