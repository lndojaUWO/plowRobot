#ifndef SCOOPDRIVE_H
#define SCOOPDRIVE_H
#include <Arduino.h>


class ScoopDrive {
public:
    ScoopDrive(int pinA, int pinB);
    ~ScoopDrive();

    int iMotorRunning; // workaround to be able to use MSE library for encoders

    void begin();
    void driveTo(long distance, long motorPosition, unsigned char pwmValue);
    void stop();

private:
    int motorPinA;
    int motorPinB;
    int ledcChannel;
    int pwmFrequency;
};

#endif // SCOOPDRIVE_H