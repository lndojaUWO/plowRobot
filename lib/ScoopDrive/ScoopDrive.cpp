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

bool ScoopDrive::driveTo(long distance, long motorPosition, long threshold, unsigned char pwmValue) {
    long difference = distance - motorPosition;

    if (difference > 0 and difference > threshold) {
        ledcWrite(ledcChannel,pwmValue);
        ledcWrite(ledcChannel+1, 0);
        return false;
    }
    else if(difference < 0 and difference < -threshold) {
        ledcWrite(ledcChannel,0);
        ledcWrite(ledcChannel+1,pwmValue);
        return false;
    }
    else{
        ledcWrite(ledcChannel,0);
        ledcWrite(ledcChannel+1,0);
        return true;
    }
}

void ScoopDrive::stop() {
    digitalWrite(ledcChannel, 0);
    digitalWrite(ledcChannel+1, 0);
}