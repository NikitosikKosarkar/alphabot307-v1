#ifndef APPLICATION_H
#define APPLICATION_H

class IServer;
class Order;

class Application {
    public:
        void sendOrder(IServer& server, Order& order);
};
#endif
