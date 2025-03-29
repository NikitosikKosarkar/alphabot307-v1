#ifndef SERVER_H
#define SERVER_H

#include "IServer.h"

class Server : public IServer {
public:
    void control(Order& order) override;
};

#endif // SERVER_H
