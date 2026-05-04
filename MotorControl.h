#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class MotorControl {
private:
    int _pinA, _pinB, _pwm1, _pwm2;
    int _speed;
    volatile long _counter;
    long _target;
    bool _isMoving;

public:
    MotorControl(int encA, int encB, int pwm1, int pwm2);
    void begin(void (*isr)());
    void move(long steps, int speed);
    void stop();
    
    // ESTA LINHA É A CHAVE: Precisa ser bool, não void
    bool update(); 

    void handleInterrupt();
    long getCount();
    bool isMoving();
};

#endif