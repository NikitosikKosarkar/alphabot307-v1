#include "Application.h"
#include <iostream>

void Application::sendOrder(Server& server, Order& order) {
    std::cout << "Application: Sending order..." << std::endl;
    server.control(order);
}
