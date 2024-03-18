#include "ScoopDrive.h"

ScoopDrive::ScoopDrive(int pinA, int pinB) {
    // Initialize member variables here if necessary
    motorPinA = pinA;
    motorPinB = pinB;
    ledcChannel = 0;
    pwmFrequency = 20000;
}

ScoopDrive::~ScoopDrive() {
    // Clean up resources here if necessary
}

void ScoopDrive::begin() {
    // Initialize the motor and LEDC here
    // For example, set the motor pins as OUTPUT and setup the LEDC
    ledcAttachPin(motorPinA, ledcChannel);
    ledcSetup(ledcChannel, pwmFrequency, 8); // 8 bit resolution
    ledcAttachPin(motorPinB,ledcChannel+1);
    ledcSetup(ledcChannel+1,pwmFrequency,8);
}

void ScoopDrive::driveTo(long distance, long motorPosition, unsigned char pwmValue) {
    if (distance > motorPosition){
        iMotorRunning = 5;
        ledcWrite(ledcChannel,pwmValue);
        ledcWrite(ledcChannel+1, 0);
    }
    else if(distance < motorPosition){
        iMotorRunning = 10;
        ledcWrite(ledcChannel,0);
        ledcWrite(ledcChannel+1,pwmValue);
    }
    else{
        iMotorRunning = 100;
        ledcWrite(ledcChannel,0);
        ledcWrite(ledcChannel+1,0);
    }
}

void ScoopDrive::stop() {
    iMotorRunning = 100;
    digitalWrite(ledcChannel, 0);
    digitalWrite(ledcChannel+1, 0);
}