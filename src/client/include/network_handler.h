#pragma once

#include "common.h"

namespace RoboComm {

class NetworkHandler : private NonCopyable {
    struct mosquitto* mqtt_client_ = nullptr;
    string current_topic_;
    string last_message_;
    mutex msg_lock_;
    bool connected_ = false;

    static void on_connect_callback(struct mosquitto *mosq, void *obj, int rc);
    static void on_message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg);
    static void on_disconnect_callback(struct mosquitto *mosq, void *obj, int rc);
    static void on_log_callback(struct mosquitto* mosq, void* obj, int level, const char* str);


public:
    NetworkHandler(const char* client_id, const char* host, int port, const char* topic);
    ~NetworkHandler();

    bool isConnected() const;
    string getCommand();
    void publishMessage(const string& topic, const string& message, int qos = 0, bool retain = false);
};

}