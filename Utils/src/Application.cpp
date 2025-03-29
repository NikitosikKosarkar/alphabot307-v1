#include "Application.h"
#include "IServer.h"
#include "Order.h"
#include <iostream>

void Application::sendOrder(IServer& server, Order& order) {
    std::cout << "Application: Sending order #" << order.getDrink().getName()
              << " to server" << std::endl;
    server.control(order);
}
