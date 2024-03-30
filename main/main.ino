#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <driveBot.h>
#include <esp_now.h>
#include <WiFi.h>

#define LEFT_MOTOR_A        35  // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B        36  // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A       37  // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B       38  // GPIO38 pin 31 (J38) Motor 2 B
#define ENCODER_LEFT_A      15  // left encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_LEFT_B      16  // left encoder B signal is connected to pin 8 GPIO16 (J16)
#define ENCODER_RIGHT_A     11  // right encoder A signal is connected to pin 19 GPIO11 (J11)
#define ENCODER_RIGHT_B     12  // right encoder B signal is connected to pin 20 GPIO12 (J12)
#define MODE_BUTTON         0   // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH 3   // DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
#define POT_R1              1   // when DIP Switch S1-3 is on, Analog AD0 (pin 39) GPIO1 is connected to Poteniometer R1
#define SMART_LED           21  // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT     1   // number of SMART LEDs in use

// all distances are written in cm
// robot important dimensions
#define PLOW_WIDTH          14.0
#define WHEEL_DIAMETER      5.0
#define PULSES_PER_REVOLUTION 1096  // encoder pulses per motor revolution
#define DISTANCE_CORRECTION 1.18
#define ROBOT_LENGTH        26.68
#define ROBOT_WIDTH         17.9
#define DISTANCE_TO_WHEELS  12.4
// sorter important dimensions
#define SCOOP_Y_OFFSET      23.5
#define SCOOP_WIDTH         17.8
#define SCOOP_ENTER_Y_OFFSET  30  // how far from the scoop the robot will go to line up

#define X_MAX               80.0
#define Y_MAX               100.0

#define KP                  50.0


unsigned int debounceTimer;                                                   
unsigned long previousMillisGyro;                                                  
unsigned long currentMillisGyro;                                                   

const int PWM_Resolution = 8;                                                  // bit resolution
const int MIN_PWM = 150;                                                       
const int MAX_PWM = pow(2, PWM_Resolution) - 1;    
float leftDriveSpeed = 0;                                                  
float rightDriveSpeed = 0;                                                 
float targetSpeed;
bool topBottomToggle = false;


unsigned int  robotModeIndex = 0;                                              // robot operational state 
enum DriveStage { STOP, DRIVE_TO_END, NEXT_COLUMN, LINE_UP, ENTER_SCOOP, EXIT_SCOOP, READY_UP};     // enums that are used so i can use text names in the switch case instead of integers
/*
This array represents the actions the robot will take in one loop. 
*/
const int driveLoopSize = 5;
DriveStage driveLoops[driveLoopSize] = {DRIVE_TO_END, NEXT_COLUMN, LINE_UP, ENTER_SCOOP, EXIT_SCOOP};
int driveLoopIndex = 0; // which stage in the driveLoop we are at
bool stageComplete;     // if True, start the next stage in the loop
int numLoops = 0;     

Adafruit_NeoPixel smartLED = Adafruit_NeoPixel(SMART_LED_COUNT, SMART_LED, NEO_GRB + NEO_KHZ800);  

// Motor, encoder, and IR objects (classes defined in MSE2202_Lib)
Motion Bot = Motion();
DriveBot driveBot = DriveBot();                                                         
Encoders LeftEncoder = Encoders();                                             
Encoders RightEncoder = Encoders();         

// MPU6050 IMU Sensor
Adafruit_MPU6050 mpu;                                                          // I2C address 0x68
float angle = PI/2.0;
bool measureAngle = false;
float yAcceleration = 0;
int setAngle = 0;

// define functions
void drive(double desiredAngle, double setSpeed, bool reverse = false, double threshold = PI/16.0);

//F4:12:FA:47:AA:D4 MAC Address
// F4:12:FA:48:19:EC RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xF4, 0x12, 0xFA, 0x48, 0x19, 0xEC};
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}



void setup() {
   Serial.begin(115200);
   smartLED.begin();                                                           
   smartLED.show(); // Initialize all pixels to 'off'

   WiFi.mode(WIFI_STA);

   if (esp_now_init() != ESP_OK){
      Serial.println("Error initializing ESP-NOW");
      return;
   }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
   
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
   numLoops = numberOfLoops();
   stageComplete = false;

   previousMillisGyro = millis();
}

void loop() {
   long motorPosition[] = {0, 0}; // represented in encouder counts
   noInterrupts();                                                             
   LeftEncoder.getEncoderRawCount();
   RightEncoder.getEncoderRawCount();
   motorPosition[0] = LeftEncoder.lRawEncoderCount;                                                 
   motorPosition[1] = (RightEncoder.lRawEncoderCount) * -1.0;
   interrupts();  
   
   readMPU();                                                           

   buttonDebounce();

   if (robotModeIndex == 0 or driveLoopIndex == 9){
         Bot.Stop("D1");
         LeftEncoder.clearEncoder();                                        
         RightEncoder.clearEncoder();
         driveBot.setPosition(getCM(ROBOT_WIDTH/2.0),getCM(DISTANCE_TO_WHEELS));
         driveBot.setDesiredX(getCM(ROBOT_WIDTH/2.0));
         driveBot.setLastEncoderValue(0,0);
         // set leds to red
         setLedColor(255, 0, 0);
         numLoops = numberOfLoops();
         driveLoopIndex = 0;
         robotModeIndex = 0;
         measureAngle = false;
         angle = PI/2.0;
   }
   else{
      measureAngle = true;
      targetSpeed = 80;  
   
      switch(driveLoops[driveLoopIndex]) {
         case DRIVE_TO_END:
            // set leds to green
            setLedColor(0, 255, 0);
            long desiredY;
            topBottomToggle == false ? desiredY = getCM(Y_MAX-(ROBOT_LENGTH-DISTANCE_TO_WHEELS)) : desiredY = getCM(DISTANCE_TO_WHEELS);
            driveBot.setDesiredY(desiredY);
            drive(driveBot.getNewAngle(), targetSpeed);
            driveBot.updatePosition(motorPosition[0], motorPosition[1], angle);
            if (driveBot.getDistance() < getCM(1)){
               Bot.Stop("D1");
               stageComplete = true;
            }
            break;
         case NEXT_COLUMN:
            // set leds to yellow
            setLedColor(255, 255, 0);
            driveBot.setDesiredX(getCM(PLOW_WIDTH * (numberOfLoops() - numLoops+1)+ROBOT_WIDTH/2.0));
            drive(driveBot.getNewAngle(), targetSpeed, false, PI/32.0);
            driveBot.updatePosition(motorPosition[0], motorPosition[1], angle);
            if (driveBot.getDistance() < getCM(1)){
               Bot.Stop("D1");
               stageComplete = true;
            }
            break;
         // case READY_UP:
         //    // set leds to orange
         //    setLedColor(255, 165, 0);
         //    driveBot.setDesiredX(getCM(X_MAX/2.0));
         //    driveBot.setDesiredY(getCM(SCOOP_Y_OFFSET + SCOOP_ENTER_Y_OFFSET));
         //    drive(driveBot.getNewAngle(), targetSpeed);
         //    driveBot.updatePosition(motorPosition[0], motorPosition[1], angle);
         //    if (driveBot.getDistance() < getCM(1)){
         //       Bot.Stop("D1");
         //       stageComplete = true;
         //    }
         //    break;
         case LINE_UP:
            // set leds to blue
            setLedColor(0, 0, 255);
            measureAngle = true;
            driveBot.setDesiredY(getCM(SCOOP_Y_OFFSET + SCOOP_ENTER_Y_OFFSET));
            driveBot.setDesiredX(getCM(X_MAX - (SCOOP_WIDTH/2.0)));
            drive(driveBot.getNewAngle(), targetSpeed);
            driveBot.updatePosition(motorPosition[0], motorPosition[1], angle);
            if (driveBot.getDistance() < getCM(0.5)){
               Bot.Stop("D1");
               stageComplete = true;
            }
            break;
         case ENTER_SCOOP:
            // set leds to purple
            setLedColor(255, 0, 255);
            measureAngle = true;
            driveBot.setDesiredY(getCM(SCOOP_Y_OFFSET+(ROBOT_LENGTH-DISTANCE_TO_WHEELS)));
            drive(3.0*PI/2.0, targetSpeed, false,PI/32.0);
            driveBot.updatePosition(motorPosition[0], motorPosition[1], angle);
            if (driveBot.getDistance() < getCM(1.0)){
               measureAngle = false;
               Bot.Stop("D1");
               stageComplete = true;
            }
            break;
         case EXIT_SCOOP:
            // set leds to white
            setLedColor(255, 255, 255);
            measureAngle = true;
            driveBot.setDesiredY(getCM(SCOOP_Y_OFFSET + SCOOP_ENTER_Y_OFFSET));
            drive(driveBot.getNewAngle(), targetSpeed, true, PI/32.0);
            driveBot.updatePosition(motorPosition[0], motorPosition[1], angle);
            if (driveBot.getDistance() < getCM(0.5)){
               measureAngle = false;
               Bot.Stop("D1");
               bool command = true;
               Serial.println(esp_now_send(peerInfo.peer_addr, (uint8_t *) &command, sizeof(command)));
               stageComplete = true;
            }
            break;
      }
      if (stageComplete){
         driveLoopIndex = nextStage(); // finds out what stage the robot must do next
      }
      
   }

}

void drive(double desiredAngle, double setSpeed, bool reverse, double threshold){
   if (reverse == true){
      desiredAngle = constrainAngle(desiredAngle - PI);
      setSpeed = -setSpeed;
   }
   float error = desiredAngle - angle;
   if (error > PI) {
   error -= 2.0 * PI;
   } else if (error < -PI) {
   error += 2.0 * PI;
   }    

   float correction;
   if (abs(error) > threshold){
      setSpeed = 0;
      correction = KP*error*2.0;
   }
   else{
      correction = KP*error;
   }

   leftDriveSpeed = setSpeed - correction;
   rightDriveSpeed = setSpeed + correction;
   leftDriveSpeed > 100 ? leftDriveSpeed = 100 : leftDriveSpeed = leftDriveSpeed;
   rightDriveSpeed > 100 ? rightDriveSpeed = 100 : rightDriveSpeed = rightDriveSpeed;
   leftDriveSpeed < -100 ? leftDriveSpeed = -100 : leftDriveSpeed = leftDriveSpeed;
   rightDriveSpeed < -100 ? rightDriveSpeed = -100 : rightDriveSpeed = rightDriveSpeed;

   if (leftDriveSpeed < 0 and rightDriveSpeed < 0){
      Bot.Reverse("D1", toPWM(abs(rightDriveSpeed)), toPWM(abs(leftDriveSpeed)));
   }
   else if (leftDriveSpeed < 0){
      Bot.Right("D1", toPWM(rightDriveSpeed),toPWM(abs(leftDriveSpeed)));
   }
   else if (rightDriveSpeed < 0){
      Bot.Left("D1", toPWM(abs(rightDriveSpeed)), toPWM(leftDriveSpeed));
   }
   else{
      Bot.Forward("D1", toPWM(rightDriveSpeed), toPWM(leftDriveSpeed));
   }
   

}


unsigned char toPWM(float speed){
   unsigned char result = map(speed, 0, 100, MIN_PWM, MAX_PWM);
   return (result);
}

void readMPU(){
   sensors_event_t a , g, temp;
   mpu.getEvent(&a, &g, &temp);

   yAcceleration = a.acceleration.y;   

   float angularVelocity = g.gyro.z;
   angularVelocity += 0.0048; // offset
   currentMillisGyro = millis();
   float changeInAngle = angularVelocity * (currentMillisGyro - previousMillisGyro) / 1000;
   changeInAngle = changeInAngle;
   measureAngle == true ? angle += changeInAngle : angle = angle;
   angle = constrainAngle(angle);
   previousMillisGyro = currentMillisGyro;
}

// returns the number of encoder counts needed to drive a certain distance
long getCM(double inCM){ 
   return((inCM * PULSES_PER_REVOLUTION*DISTANCE_CORRECTION) / (WHEEL_DIAMETER * PI));
}

double toCm(long encoderCount){
   double result = (encoderCount * WHEEL_DIAMETER * PI) / (PULSES_PER_REVOLUTION);
   return (result);
}

void setLedColor(int r, int g, int b){
   if (smartLED.getPixelColor(0) != smartLED.Color(r, g, b)){
      smartLED.setPixelColor(0, smartLED.Color(r, g, b));
      smartLED.show();
   }
}

int numberOfLoops(){
   int result = (X_MAX-SCOOP_WIDTH/2) / PLOW_WIDTH - 1;
   result % 2 != 0 ? result-- : result = result;
   return (result);
}

int nextStage(){
   if(stageComplete){
      stageComplete = false;
      driveLoopIndex++;
      numLoops == 0 and driveLoopIndex == 1 ? driveLoopIndex++ : driveLoopIndex = driveLoopIndex;
      driveLoopIndex == 2 ? topBottomToggle = not topBottomToggle : topBottomToggle = topBottomToggle;
      if (driveLoopIndex >= 2){ // if we have finished the loop
         if (numLoops > 0){
            numLoops--;
            driveLoopIndex = 0;            
         }
         else if (driveLoopIndex == driveLoopSize){
            driveLoopIndex = 0;
            return 9; // indicates that the robot is done
         }
         
      }
   }
   return driveLoopIndex;
}

double constrainAngle(double x){
    x = fmod(x,2.0*PI);
    if (x < 0)
        x += 2.0*PI;
    return x;
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



