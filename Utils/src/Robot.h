// Robot.h
#ifndef ROBOT_H
#define ROBOT_H

#include "Drink.h"

class IRobotMovement {
    public:
        virtual int start() = 0;
        virtual int stop() = 0;
        virtual void moveForward(int time) = 0;
        virtual void moveBackward(int time) = 0;
        virtual void turnLeft() = 0;
        virtual void turnRight() = 0;
        virtual ~IRobotMovement() = default;
};

class IRobotBarista {
    public:
        virtual void prepare(Drink& drink) = 0;
        virtual ~IRobotBarista() = default;
};

class Robot : public IRobotMovement, public IRobotBarista {
    public:
        int start() override;
        int stop() override;
        void moveForward(int time) override;
        void moveBackward(int time) override;
        void turnLeft() override;
        void turnRight() override;
        void prepare(Drink& drink) override;
};
#endif
