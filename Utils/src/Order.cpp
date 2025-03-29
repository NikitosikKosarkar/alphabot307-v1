#include "Order.h"

Order::Order(int id, Drink drink, std::string status) 
    : orderID(id), drink(drink), status(status) {}

Drink& Order::getDrink() {
    return drink;
}

const std::string& Order::getStatus() const {
    return status;
}

void Order::setStatus(const std::string& newStatus) {
    status = newStatus;
}
