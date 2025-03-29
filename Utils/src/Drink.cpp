#include "Drink.h"

Drink::Drink(std::string name, float price) 
    : name(std::move(name)), price(price) {}

const std::string& Drink::getName() const {
    return name;
}

float Drink::getPrice() const {
    return price;
}
