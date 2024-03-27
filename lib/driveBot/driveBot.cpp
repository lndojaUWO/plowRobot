#include "driveBot.h"
#include <math.h>

#define PI 3.14159265
#include <Arduino.h>

// Implement the member functions of the DriveBot class here

DriveBot::DriveBot() {
    // Constructor implementation
    lastEncoderValue = 0.0;
}

DriveBot::~DriveBot() {
    // Destructor implementation
}

void DriveBot::setDesiredX(long x) {
    desiredXPosition = x;
}

void DriveBot::setDesiredY(long y) {
    desiredYPosition = y;
}
double DriveBot::getXPosition() {
    return xPosition;
}
double DriveBot::getYPosition() {
    return yPosition;
}
double DriveBot::getDesiredXPosition() {
    return desiredXPosition;
}
double DriveBot::getDesiredYPosition() {
    return desiredYPosition;
}

void DriveBot::setPosition(long x, long y) {
    xPosition = x;
    yPosition = y;
}

void DriveBot::updatePosition(long leftMotor, long rightMotor, float angle) {
    long encoderValue = (leftMotor + rightMotor) / 2.0;
    double distance = encoderValue - lastEncoderValue;
    xPosition += distance * cos(angle);
    yPosition += distance * sin(angle);
    lastEncoderValue = encoderValue;  
}

void DriveBot::setLastEncoderValue(long leftMotor, long rightMotor) {
    lastEncoderValue = (leftMotor + rightMotor) / 2.0;
}

double DriveBot::getNewAngle() {
    if (desiredXPosition - xPosition == 0) {
        desiredYPosition - yPosition > 0 ? desiredAngle = PI/2.0 : desiredAngle = 3.0*PI/2.0;
    }
    else{
        desiredAngle = atan2((desiredYPosition - yPosition), (desiredXPosition - xPosition));
    }
    desiredAngle = constrainAngle(desiredAngle);
    return (desiredAngle);
}

long DriveBot::getDistance() {
    return sqrt(pow(desiredXPosition - xPosition, 2) + pow(desiredYPosition - yPosition, 2));
}

double DriveBot::constrainAngle(double x){
    x = fmod(x,2.0*PI);
    if (x < 0)
        x += 2.0*PI;
    return x;
}
