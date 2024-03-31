#include "ScoopDrive.h"

ScoopDrive::ScoopDrive(int pinA, int pinB) {
    motorPinA = pinA;
    motorPinB = pinB;
    ledcChannel = 4;
    pwmFrequency = 20000;
}

ScoopDrive::~ScoopDrive() {
}

void ScoopDrive::begin() {
    // Initialize the motor and LEDC here
    ledcAttachPin(motorPinA, ledcChannel);
    ledcSetup(ledcChannel, pwmFrequency, 8); // 8 bit resolution
    ledcAttachPin(motorPinB,ledcChannel+1);
    ledcSetup(ledcChannel+1,pwmFrequency,8);
}

bool ScoopDrive::driveTo(long distance, long motorPosition, long threshold, unsigned char pwmValue) {
    long difference = distance - motorPosition;

    // determines if the motor should move forward or backward to go towards the desired position
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
    else{  // if the motor is within the threshold, stop the motor
        ledcWrite(ledcChannel,0);
        ledcWrite(ledcChannel+1,0);
        return true;
    }
}

void ScoopDrive::stop() {
    digitalWrite(ledcChannel, 0);
    digitalWrite(ledcChannel+1, 0);
}