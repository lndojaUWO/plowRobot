#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define LEFT_MOTOR_A        35                                                 // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B        36                                                 // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A       37                                                 // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B       38                                                 // GPIO38 pin 31 (J38) Motor 2 B
#define ENCODER_LEFT_A      15                                                 // left encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_LEFT_B      16                                                 // left encoder B signal is connected to pin 8 GPIO16 (J16)
#define ENCODER_RIGHT_A     11                                                 // right encoder A signal is connected to pin 19 GPIO11 (J11)
#define ENCODER_RIGHT_B     12                                                 // right encoder B signal is connected to pin 20 GPIO12 (J12)
#define MODE_BUTTON         0                                                  // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH 3                                                  // DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
#define POT_R1              1                                                  // when DIP Switch S1-3 is on, Analog AD0 (pin 39) GPIO1 is connected to Poteniometer R1
#define SMART_LED           21                                                 // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT     1                                                  // number of SMART LEDs in use

// all distances are written in cm
#define TURN90DISTANCE      12                                                 
#define PLOW_WIDTH          15                                                 

#define X_MAX               30
#define Y_MAX               80

#define KP                  5
#define KI                  0
#define KD                  1   
int integral = 0;        
float derivative = 0;
float lastError = 0;


unsigned int debounceTimer;                                                   
unsigned long previousMillisGyro;                                                  
unsigned long currentMillisGyro;                                                   

const int PWM_Resolution = 8;                                                  // bit resolution
const int MIN_PWM = 150;                                                       
const int MAX_PWM = pow(2, PWM_Resolution) - 1;    
const int PULSES_PER_REVOLUTION = 1096;                                        // encoder pulses per motor revolution
unsigned char leftDriveSpeed = 0;                                                  
unsigned char rightDriveSpeed = 0;                                                 
float targetSpeed;

const int WHEEL_DIAMETER = 5;
unsigned int  robotModeIndex = 0;                                              // robot operational state 
enum DriveStage { STOP, DRIVE_TO_END, TURN_LEFT, NEXT_COLUMN,TURN_RIGHT };     // enums that are used so i can use text names in the switch case instead of integers
/*
This array represents the actions the robot will take in one loop. 
*/
DriveStage driveLoops[9] = {STOP, DRIVE_TO_END, TURN_LEFT, NEXT_COLUMN, TURN_LEFT, DRIVE_TO_END, TURN_RIGHT, NEXT_COLUMN, TURN_RIGHT};
int driveLoopIndex = 0; // which stage in the driveLoop we are at
bool stageComplete;     // if True, start the next stage in the loop
int numLoops = 0;     

Adafruit_NeoPixel smartLED = Adafruit_NeoPixel(SMART_LED_COUNT, SMART_LED, NEO_GRB + NEO_KHZ800);  

// Motor, encoder, and IR objects (classes defined in MSE2202_Lib)
Motion Bot = Motion();                                                         
Encoders LeftEncoder = Encoders();                                             
Encoders RightEncoder = Encoders();         

// MPU6050 IMU Sensor
Adafruit_MPU6050 mpu;                                                          // I2C address 0x68
float angle = 0;
bool measureAngle = false;
float yAcceleration = 0;

void setup() {
   Serial.begin(115200);
   smartLED.begin();                                                           
   smartLED.show(); // Initialize all pixels to 'off'
   
  // Set up motors and encoders
   Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B); 
   LeftEncoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B, &Bot.iLeftMotorRunning ); 
   RightEncoder.Begin(ENCODER_RIGHT_A, ENCODER_RIGHT_B, &Bot.iRightMotorRunning ); 
   pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);
   pinMode(MODE_BUTTON, INPUT_PULLUP);

  Wire.begin(47,48);
   // connect to mpu
   if (!mpu.begin()) {
      Serial.println("Failed to find MPU6050 chip");
      while (1) {
         delay(10);
      }
   }

   mpu.setGyroRange(MPU6050_RANGE_250_DEG);
   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);


   debounceTimer = 0;
   //numLoops = numberOfLoops(X_MAX, PLOW_WIDTH);
   numLoops = 1;
   stageComplete = false;

   previousMillisGyro = millis();
}

void loop() {
   long motorPosition[] = {0, 0}; // represented in encouder counts
   int potentiometer = 0; // raw ADC value from potentiometer

   noInterrupts();                                                             
   LeftEncoder.getEncoderRawCount();
   RightEncoder.getEncoderRawCount();
   motorPosition[0] = LeftEncoder.lRawEncoderCount;                                                 
   motorPosition[1] = RightEncoder.lRawEncoderCount;
   interrupts();                                                               

   buttonDebounce();
   readMPU();

  


   if (robotModeIndex == 0 or driveLoopIndex == 9){
         Bot.Stop("D1");
         LeftEncoder.clearEncoder();                                        
         RightEncoder.clearEncoder();
         // set leds to red
         setLedColor(255, 0, 0);
         //numLoops = numberOfLoops(X_MAX, PLOW_WIDTH);
         numLoops = 1;
         driveLoopIndex = 0;
         robotModeIndex = 0;
   }
   else{
      potentiometer = analogRead(POT_R1);
      targetSpeed = map(potentiometer, 0, 4095, MIN_PWM, MAX_PWM);


      /*
      This switch case determines what the robot will do in each stage
      for reference one loop is: 

      STOP, DRIVE_TO_END, TURN_LEFT, NEXT_COLUMN, TURN_LEFT, DRIVE_TO_END, TURN_RIGHT, NEXT_COLUMN, TURN_RIGHT
      
      */
      switch(driveLoops[driveLoopIndex]) {
         case STOP:
            Bot.Stop("D1");
            LeftEncoder.clearEncoder();                                        
            RightEncoder.clearEncoder();
            // set leds to red
            setLedColor(255, 0, 0);
            stageComplete = true;
            break;
         case DRIVE_TO_END:
            // set leds to green
            setLedColor(0, 255, 0);
            if (driveTo(Y_MAX, motorPosition[0])){
                  stageComplete = true;                  
            }
            break;
         case TURN_LEFT:
            // set leds to blue
            setLedColor(0, 0, 255);
            if (turnTo(90, false)){
                  stageComplete = true;
            }
            break;
         case NEXT_COLUMN:
            // set leds to yellow
            setLedColor(255, 255, 0);
            if (driveTo(PLOW_WIDTH, motorPosition[0])){
                  stageComplete = true;
            }
            break;
         case TURN_RIGHT:
            // set leds to blue
            setLedColor(0, 0, 255);
            if (turnTo(-90, true)){
                  stageComplete = true;
            }
            break;
      }
      if (stageComplete){
         driveLoopIndex = nextStage(); // finds out what stage the robot must do next
      }
   }
}

void straightDriveSpeeds(){

   float error = angle;
   error < 0.01 and error > -0.05 ? integral = 0 : integral += error;
   derivative = error - lastError;
   lastError = error;

   int correction = (KP * error + KI * integral + KD * derivative) * -1;
   leftDriveSpeed = targetSpeed + correction;
   rightDriveSpeed = targetSpeed - correction;

   // max drive speed cap
   leftDriveSpeed > MAX_PWM ? leftDriveSpeed = MAX_PWM : leftDriveSpeed = leftDriveSpeed;
   rightDriveSpeed > MAX_PWM ? rightDriveSpeed = MAX_PWM : rightDriveSpeed = rightDriveSpeed;
   // min drive speed cap
   leftDriveSpeed < MIN_PWM ? leftDriveSpeed = MIN_PWM : leftDriveSpeed = leftDriveSpeed;
   rightDriveSpeed < MIN_PWM ? rightDriveSpeed = MIN_PWM : rightDriveSpeed = rightDriveSpeed;

   // display the PID values to Serial
   Serial.print("Error: ");
   Serial.print(error*KP);
   Serial.print(" Integral: ");
   Serial.print(integral*KI);
   Serial.print(" Derivative: ");
   Serial.print(derivative*KD);
   Serial.print(" Correction: ");
   Serial.print(correction);
   Serial.print(" Drive Speeds: ");
   Serial.print(leftDriveSpeed);
   Serial.print(" ");
   Serial.println(rightDriveSpeed);
   
        

}

void readMPU(){
   sensors_event_t a , g, temp;
   mpu.getEvent(&a, &g, &temp);

   yAcceleration = a.acceleration.y;   

   float angularVelocity = g.gyro.z;
   angularVelocity += 0.0048; // offset
   currentMillisGyro = millis();
   float changeInAngle = angularVelocity * (currentMillisGyro - previousMillisGyro) / 1000;
   changeInAngle = RAD_TO_DEG * changeInAngle;
   measureAngle == true ? angle += changeInAngle : angle = 0;
   previousMillisGyro = currentMillisGyro;
}

bool turnTo(int setAngle, bool turningRight){
   if(abs(angle) < abs(setAngle)){
      measureAngle = true;
      turningRight == true ? Bot.Left("D1", targetSpeed, targetSpeed*1.25) : Bot.Right("D1", targetSpeed, targetSpeed); // misleading, bot.left turns it to the right
      return false;
   }
   else{
      measureAngle = false;
      Bot.Stop("D1");
      LeftEncoder.clearEncoder();
      RightEncoder.clearEncoder();
      return true;
   }
}

// returns the number of encoder counts needed to drive a certain distance
long getCM(long inCM){ 
   return(inCM * PULSES_PER_REVOLUTION / (WHEEL_DIAMETER * 3.14159));
}

// input a distance in cm and it will drive forward that amount
bool driveTo(int distance, long motorPosition){

   straightDriveSpeeds();

   if(motorPosition < getCM(distance)){ 
      measureAngle = true;
      Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed);  
      return false;
   }
   else{
      measureAngle = false;
      Bot.Stop("D1");
      LeftEncoder.clearEncoder();
      RightEncoder.clearEncoder();
      return true;
   }
}

void setLedColor(int r, int g, int b){
   if (smartLED.getPixelColor(0) != smartLED.Color(r, g, b)){
      smartLED.setPixelColor(0, smartLED.Color(r, g, b));
      smartLED.show();
   }
}

int numberOfLoops(int xMax, int plowWidth){
   return xMax / plowWidth;
}

int nextStage(){
   if(stageComplete){
      stageComplete = false;
      driveLoopIndex++;
      if (driveLoopIndex == 9){ // if we have finished the loop
         if (numLoops == 0){
            return 9; // if we have done all the loops we return 9 to indicate that we are done
         }
         else{
            numLoops--;
            driveLoopIndex = 0;
         }
      }
   }
   return driveLoopIndex;
}

// placed into the depths so we dont have to look at it
// reads the button and toggles the robot mode
void buttonDebounce(){
    // Mode pushbutton debounce and toggle
   if (!digitalRead(MODE_BUTTON)) {                                         // if pushbutton GPIO goes LOW (nominal push)
      // Start debounce
      if (debounceTimer <= 25) {                                           // 25 millisecond debounce time
         debounceTimer = debounceTimer + 1;                               // increment debounce timer count
         if (debounceTimer > 25) {                                         // if held for at least 25 mS
            debounceTimer = 1000;                                          // change debounce timer count to 1 second
         }
      }
      if (debounceTimer >= 1000) {                                         // maintain 1 second timer count until release
         debounceTimer = 1000;
      }
   }
   else {                                                                   // pushbutton GPIO goes HIGH (nominal release)
      if(debounceTimer <= 26) {                                            // if release occurs within debounce interval
         debounceTimer = 0;                                                // reset debounce timer count
      }
      else {
         debounceTimer = debounceTimer + 1;                               // increment debounce timer count
         if(debounceTimer >= 1025) {                                       // if pushbutton was released for 25 mS
            debounceTimer = 0;                                             // reset debounce timer count
            robotModeIndex == 0 ? robotModeIndex = 1 : robotModeIndex = 0; // toggle robot operational state
         }
      }
   }
}



