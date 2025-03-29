#ifndef SERVER_H
#define SERVER_H

#include "Order.h"

class IServer {
    public:
        virtual void control(Order& order) = 0;
        virtual ~IServer() = default;
};

class Server : public IServer {
    public:
        void control(Order& order) override;
};
#endif
