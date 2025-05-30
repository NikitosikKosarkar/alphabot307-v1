#include "network_handler.h"

namespace RoboComm {

NetworkHandler::NetworkHandler(const char* client_id, const char* host, int port, const char* topic)
    : current_topic_(topic) {
    mosquitto_lib_init();
    mqtt_client_ = mosquitto_new(client_id, true, this);

    if (!mqtt_client_) {
        throw runtime_error("MQTT init failed: mosquitto_new returned null.");
    }

    mosquitto_connect_callback_set(mqtt_client_, on_connect_callback);
    mosquitto_message_callback_set(mqtt_client_, on_message_callback);
    mosquitto_disconnect_callback_set(mqtt_client_, on_disconnect_callback);
    mosquitto_log_callback_set(mqtt_client_, on_log_callback);


    int ret = mosquitto_connect(mqtt_client_, host, port, MQTT_KEEP_ALIVE_S);
    if (ret != MOSQ_ERR_SUCCESS) {
        mosquitto_destroy(mqtt_client_);
        mqtt_client_ = nullptr;
        mosquitto_lib_cleanup();
        throw runtime_error("MQTT connect failed: " + string(mosquitto_strerror(ret)));
    }

    ret = mosquitto_loop_start(mqtt_client_);
    if (ret != MOSQ_ERR_SUCCESS) {
        mosquitto_destroy(mqtt_client_);
        mqtt_client_ = nullptr;
        mosquitto_lib_cleanup();
        throw runtime_error("MQTT loop_start failed: " + string(mosquitto_strerror(ret)));
    }
}

NetworkHandler::~NetworkHandler() {
    if (mqtt_client_) {
        mosquitto_loop_stop(mqtt_client_, true);
        if(connected_) {
            mosquitto_disconnect(mqtt_client_);
        }
        mosquitto_destroy(mqtt_client_);
    }
    mosquitto_lib_cleanup();
    cout << "NetworkHandler cleaned up." << endl;
}

void NetworkHandler::on_connect_callback(struct mosquitto *mosq, void *obj, int rc) {
    NetworkHandler* self = static_cast<NetworkHandler*>(obj);
    if (rc == 0) {
        cout << "Connected to MQTT broker." << endl;
        self->connected_ = true;
        int ret = mosquitto_subscribe(self->mqtt_client_, nullptr, self->current_topic_.c_str(), 1);
        if (ret != MOSQ_ERR_SUCCESS) {
            cerr << "Subscription to topic '" << self->current_topic_ << "' failed: " << mosquitto_strerror(ret) << endl;
        } else {
            cout << "Subscribed to topic: " << self->current_topic_ << endl;
        }
    } else {
        cerr << "MQTT Connect failed with code " << rc << ": " << mosquitto_strerror(rc) << endl;
        self->connected_ = false;
    }
}

void NetworkHandler::on_message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg) {
    NetworkHandler* self = static_cast<NetworkHandler*>(obj);
    if (msg->payloadlen) {
        lock_guard<mutex> guard(self->msg_lock_);
        self->last_message_ = string(static_cast<char*>(msg->payload), msg->payloadlen);
         cout << "Received message on topic " << msg->topic << ": " << self->last_message_ << endl;
    } else {

    }
}

void NetworkHandler::on_disconnect_callback(struct mosquitto *mosq, void *obj, int rc) {
    NetworkHandler* self = static_cast<NetworkHandler*>(obj);
    self->connected_ = false;
    cerr << "Disconnected from MQTT broker with code " << rc << "." << endl;
    if (rc != 0) {

    }
}

void NetworkHandler::on_log_callback(struct mosquitto* mosq, void* obj, int level, const char* str) {



}


bool NetworkHandler::isConnected() const {
    return connected_;
}

string NetworkHandler::getCommand() {
    lock_guard<mutex> lock(msg_lock_);
    if (!last_message_.empty()) {
        string cmd = last_message_;
        last_message_.clear();
        return cmd;
    }
    return "";
}


void NetworkHandler::publishMessage(const string& topic, const string& message, int qos, bool retain) {
    if (!connected_) {
        cerr << "Cannot publish, not connected to MQTT broker." << endl;
        return;
    }
    int ret = mosquitto_publish(mqtt_client_, nullptr, topic.c_str(), message.length(), message.c_str(), qos, retain);
    if (ret != MOSQ_ERR_SUCCESS) {
        cerr << "Failed to publish message to topic '" << topic << "': " << mosquitto_strerror(ret) << endl;
    } else {

    }
}

}