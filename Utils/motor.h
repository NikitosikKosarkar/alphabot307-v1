#ifndef MOTOR_H
#define MOTOR_H

class Motor {
public:
    Motor();
    ~Motor();
    int start();
    int stop();
    void moveForward(int time);
    void moveBackward(int time);
    void moveRight();
    void moveLeft();
};

#endif // MOTOR_H