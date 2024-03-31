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

// takes in the distance the robot has traveled and the angle it is facing and then using trig to determine its x,y change in position
void DriveBot::updatePosition(long leftMotor, long rightMotor, float angle) {
    long encoderValue = (leftMotor + rightMotor) / 2.0; // gets the average of the two encoder values
    double distance = encoderValue - lastEncoderValue;
    xPosition += distance * cos(angle);
    yPosition += distance * sin(angle);
    lastEncoderValue = encoderValue;  
}

void DriveBot::setLastEncoderValue(long leftMotor, long rightMotor) {
    lastEncoderValue = (leftMotor + rightMotor) / 2.0;
}

double DriveBot::getNewAngle() {
    if (desiredXPosition - xPosition == 0) { // avoids division by 0
        desiredYPosition - yPosition > 0 ? desiredAngle = PI/2.0 : desiredAngle = 3.0*PI/2.0;
    }
    else{
        // calculates the angle the robot needs to turn to face the desired position
        desiredAngle = atan2((desiredYPosition - yPosition), (desiredXPosition - xPosition)); 
    }
    desiredAngle = constrainAngle(desiredAngle);
    return (desiredAngle);
}

long DriveBot::getDistance() {
    return sqrt(pow(desiredXPosition - xPosition, 2) + pow(desiredYPosition - yPosition, 2));
}

double DriveBot::constrainAngle(double angleToConstrain){
    angleToConstrain = fmod(angleToConstrain,2.0*PI);
    if (angleToConstrain < 0)
        angleToConstrain += 2.0*PI;
    return angleToConstrain;
}
