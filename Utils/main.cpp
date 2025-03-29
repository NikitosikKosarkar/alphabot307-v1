#include "src/Application.h"
#include "src/Student.h"
#include "src/Drink.h"
#include "src/Order.h"
#include "src/Server.h"

int main() {
    Student student;
    Application app;
    Server server;

    student.useApp();

    Drink coffee("Latte", 3.50f);
    Order order(1, coffee, "New");

    app.sendOrder(server, order);

    return 0;
}