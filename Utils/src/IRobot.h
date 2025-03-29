#ifndef IROBOT_H
#define IROBOT_H

class IRobot {
    public:
        virtual ~IRobot() = default;
        virtual void prepareDrink() = 0;
        virtual void deliverDrink() = 0;
};

#endif // IROBOT_H
