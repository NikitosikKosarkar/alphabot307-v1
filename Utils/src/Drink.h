#ifndef DRINK_H
#define DRINK_H

#include <string>

class Drink {
    std::string name;
    float price;
    public:
        Drink(std::string name, float price);
        const std::string& getName() const;
        float getPrice() const;
};
#endif
