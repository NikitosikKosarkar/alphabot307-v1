#ifndef ORDER_H
#define ORDER_H

#include "Drink.h"
#include <string>

class Order {
    int orderID;
    Drink drink;
    std::string status;
    public:
        Order(int id, Drink drink, std::string status);
        Drink& getDrink();
        const std::string& getStatus() const;
        void setStatus(const std::string& newStatus);
};
#endif
