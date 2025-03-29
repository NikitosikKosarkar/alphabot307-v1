#include "Server.h"
#include "Order.h"
#include <iostream>

void Server::control(Order& order) {
    std::cout << "Server: Processing order for " << order.getDrink().getName() << std::endl;
    order.setStatus("Processing");

    mqttClient.publish("robot/commands", "prepare " + order.getDrink().getName());
    order.setStatus("Prepared");

    mqttClient.publish("robot/commands", "start");
    mqttClient.publish("robot/commands", "move_forward 5");
    mqttClient.publish("robot/commands", "stop");
    order.setStatus("Delivered");

    std::cout << "Server: Order status - " << order.getStatus() << std::endl;
}
