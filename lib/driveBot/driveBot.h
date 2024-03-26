#ifndef DRIVEBOT_H
#define DRIVEBOT_H
#include <Arduino.h>

// Include any necessary libraries here

class DriveBot {
public:
    // Constructor
    DriveBot();

    // Destructor
    ~DriveBot();

    // Add your public member functions here
    void setPosition(long leftMotor, long rightMotor);
    void updatePosition(long leftMotor, long rightMotor, float angle, bool isTurning);
    void setDesiredX(long x);
    void setDesiredY(long y);
    void setLastEncoderValue(long leftMotor, long rightMotor);
    double getNewAngle();
    long getDistance();
    double getXPosition();
    double getYPosition();
    double getDesiredXPosition();
    double getDesiredYPosition();

private:
    // Add your private member variables and functions here
    long lastEncoderValue;
    double xPosition;
    double yPosition;
    long desiredXPosition;
    long desiredYPosition;
    double desiredAngle;
    double constrainAngle(double x);

};

#endif // DRIVEBOT_H