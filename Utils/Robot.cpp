#include "Robot.h"
#include <iostream>

void Robot::prepare(Drink &drink) {
    std::cout << "Robot prepares " << drink.getName() << std::endl;
}
