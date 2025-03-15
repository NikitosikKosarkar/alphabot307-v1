#ifndef DRINK_H
#define DRINK_H
#include <string>

class Drink {
    std::string name;
    float price;
    public:
        Drink(std::string name, float price): name(name), price(price) {};
        std::string getName() {
            return name;
        };
}

#endif //DRINK_H
