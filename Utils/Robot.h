#ifndef ROBOT_H
#define ROBOT_H
#include "Drink.h"

class Robot {
    public:
        void prepare(Drink& drink);

        int start();
        int stop();
        void moveForward(int time);
        void moveBackward(int time);
        void turnRight();
        void turnLeft();
};

#endif //ROBOT_H
