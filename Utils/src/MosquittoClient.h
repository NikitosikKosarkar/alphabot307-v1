#ifndef MOSQUITTOCLIENT_H
#define MOSQUITTOCLIENT_H

#include "mosquitto.h"
#include <string>

class MosquittoClient {
    struct mosquitto *mosquitto;
    static void on_message(struct mosquitto *mosquitto, void *userdata, const struct mosquitto_message *message);
    public:
        MosquittoClient();
        ~MosquittoClient();
        void publish(const std::string topic, const std::string message);
        void subscribe(const std::string topic);
};

#endif //MOSQUITTOCLIENT_H