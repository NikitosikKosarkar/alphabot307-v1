#include "Server.h"
#include "Order.h"
#include "Robot.h"
#include <iostream>

void Server::control(Order& order) {
    std::cout << "Server: Processing order for " << order.getDrink().getName() << std::endl;
    order.setStatus("Processing");

    Robot robot;
    robot.prepare(order.getDrink());
    order.setStatus("Prepared");

    robot.start();
    robot.moveForward(5);
    robot.stop();
    order.setStatus("Delivered");

    std::cout << "Server: Order status - " << order.getStatus() << std::endl;
}
