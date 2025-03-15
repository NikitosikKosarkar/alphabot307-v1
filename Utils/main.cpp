#include "Application.h"
#include "Server.h"
#include "Robot.h"
#include "Drink.h"
#include "Order.h"

int main() {
    Application app;
    Server server;
    Robot robot;

    Drink coffee("Coffee", 2.5);
    Order order(1, coffee, "Pending");

    app.sendOrder(server, order);
    server.control(robot);
    robot.prepare(order.getDrink());

    return 0;
}