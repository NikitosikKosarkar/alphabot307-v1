#ifndef ORDER_H
#define ORDER_H
#include "Drink.h"

class Order {
    int orderID;
    Drink drink;
    std::string status;
    public:
        Order(int id, Drink drink, std::string status): orderID(id), drink(drink), status(status) {};
        Drink& getDrink() {
            return drink;
        };
};

#endif //ORDER_H
