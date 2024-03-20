#ifndef SCOOPDRIVE_H
#define SCOOPDRIVE_H
#include <Arduino.h>


class ScoopDrive {
public:
    ScoopDrive(int pinA, int pinB);
    ~ScoopDrive();

    void begin();
    bool driveTo(long distance, long motorPosition, long threshold, unsigned char pwmValue);
    void stop();

private:
    int motorPinA;
    int motorPinB;
    int ledcChannel;
    int pwmFrequency;
};

#endif // SCOOPDRIVE_H