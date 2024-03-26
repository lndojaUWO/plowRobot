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

void DriveBot::updatePosition(long leftMotor, long rightMotor, float angle, bool isTurning) {
    long encoderValue = (leftMotor + rightMotor) / 2.0;
    double distance = encoderValue - lastEncoderValue;
    // if (isTurning == false){
        xPosition += distance * cos(angle);
        yPosition += distance * sin(angle);
    // }
    //Serial.println(distance);
    // print left motor, right motor, and encoder values
    // Serial.print("Left Motor: ");
    // Serial.print(leftMotor);
    // Serial.print(" Right Motor: ");
    // Serial.print(rightMotor);
    // Serial.print(" Encoder Value: ");
    // Serial.print(encoderValue);
    // Serial.print(" Last Encoder Value: ");
    // Serial.print(lastEncoderValue);
    
    lastEncoderValue = encoderValue;  

    //print positions
    // Serial.print( "X: ");
    // Serial.print(xPosition);
    // Serial.print(" Y: ");
    // Serial.print(yPosition);
    // Serial.print(" Desired X: ");
    // Serial.print(desiredXPosition);
    // Serial.print(" Desired Y: ");
    // Serial.print(desiredYPosition);
    // Serial.print(" Current Angle: ");
    // Serial.print(angle);
    // Serial.print(" Desired Angle: ");
    // Serial.println(desiredAngle);

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
