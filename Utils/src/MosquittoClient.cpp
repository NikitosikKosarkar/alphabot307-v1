#include "MosquittoClient.h"
#include <iostream>

MosquittoClient::MosquittoClient() {
    mosquitto_lib_init();
    mosquitto = mosquitto_new("client", true, this);
    if (mosquitto) {
        mosquitto_connect(mosquitto, "localhost", 1883, 60);
        mosquitto_message_callback_set(mosquitto, on_message);
        mosquitto_loop_start(mosquitto);
    }
}

MosquittoClient::~MosquittoClient() {
    if (mosquitto) {
        mosquitto_disconnect(mosquitto);
        mosquitto_destroy(mosquitto);
    }
    mosquitto_lib_cleanup();
}

void MosquittoClient::publish(const std::string& topic, const std::string& message) {
    mosquitto_publish(mosquitto, nullptr, topic.c_str(), message.size(), message.c_str(), 0, false);
}

void MosquittoClient::subscribe(const std::string& topic) {
    mosquitto_subscribe(mosquitto, nullptr, topic.c_str(), 0);
}

void MosquittoClient::on_message(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message) {
    std::cout << "MQTT Message received on topic " << message->topic << ": " << (char*)message->payload << std::endl;
}
