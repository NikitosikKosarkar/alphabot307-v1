#ifndef SERVER_H
#define SERVER_H

#include "IServer.h"
#include "MosquittoClient.h"

class Server : public IServer {
    MosquittoClient mqttClient;
    public:
        void control(Order& order) override;
};

#endif // SERVER_H
