#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TCS34725.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <ScoopDrive.h>
#include <WiFi.h>
#include <esp_now.h>

#define MOTOR_A             35  // GPIO35 pin 28 (J35) Motor 1 A
#define MOTOR_B             36  // GPIO36 pin 29 (J36) Motor 1 B
#define ENCODER_A           15  // encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_B           16  // encoder B signal is connected to pin 8 GPIO16 (J16)
#define MODE_BUTTON          0  // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH  3  // DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
#define POT_R1               1  // when DIP Switch S1-3 is on, Analog AD0 (pin 39) GPIO1 is connected to Poteniometer R1
#define SMART_LED           21  // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT      1  // number of SMART LEDs in use
#define commonAnode       true  // set to false if using a common cathode LED
#define SERVO_PIN           42
#define SERVO_PAUSE_TIME  2500  // time in milliseconds that the servo will pause in between movements

void ARDUINO_ISR_ATTR encoderISR(void* arg);

unsigned int robotModeIndex = 0; // robot operational state 
unsigned int debounceTimer;         

unsigned int previousMillisColour;
unsigned int currentMillisColour;
unsigned int previousMillisServo;
unsigned int currentMillisServo;
unsigned int previousMillisServoVibrate;
unsigned int previousMillisStage;

// DC Motor and Encoder Constants
const int PWM_Resolution = 8;                                                  
const int MIN_PWM = 150;                                                       
const int MAX_PWM = pow(2, PWM_Resolution) - 1;    
const int PULSES_PER_REVOLUTION = 1096;                                        
const int frequency = 20000;

// Encoder structure
struct Encoder {
   const int channalA; // GPIO pin for encoder channel A
   const int channalB; // GPIO pin for encoder channel B
   long position; // current encoder position
};

// Create instances of the classes
Encoder encoder = {ENCODER_A, ENCODER_B, 0}; // Create an encoder object
Adafruit_NeoPixel smartLED = Adafruit_NeoPixel(SMART_LED_COUNT, SMART_LED, NEO_GRB + NEO_KHZ800);  
ScoopDrive scoopMotor = ScoopDrive(MOTOR_A,MOTOR_B);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Servo sorterServo;

byte gammatable[256];
float red, green, blue; // for color sensor
int servoPos = 0;
char result = 'w';
bool servoToggle = false;
bool scoopVibrateToggle = false;

enum stage {ROTATE_SCOOP, START_SORTER, STOP}; 
stage sorterStage = ROTATE_SCOOP;

const float greenValue = 0.39;
// 0.37
// F4:12:FA:48:19:EC MAC Address

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
   bool command = false;
   memcpy(&command, incomingData, sizeof(command));
   if (command == true){
         robotModeIndex = 1;
   }


}

void setup() {
   smartLED.begin();                                                           
   smartLED.show(); // Initialize all pixels to 'off'
   ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);


   // color sensor setup
   Wire.begin(47,48);
   if (tcs.begin()) {
      //Serial.println("Found sensor");
   } else {
      // Serial.println("No TCS34725 found ... check your connections");
      while (1); // halt!
   }
   
   
   // motor and encoder setup
   scoopMotor.begin();
   pinMode(encoder.channalA, INPUT);                   
   pinMode(encoder.channalB, INPUT);                   
   attachInterruptArg(encoder.channalA, encoderISR, &encoder, RISING); // configure encoder to trigger interrupt with each rising edge on channel A

   // button and motor enable switch setup
   pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);
   pinMode(MODE_BUTTON, INPUT_PULLUP);

   debounceTimer = 0;

   // Color sensor gamma correction, dont ask me how it works i have no clue
   for (int i=0; i<256; i++) {
      float x = i;
      x /= 255;
      x = pow(x, 2.5);
      x *= 255;

      if (commonAnode) {
         gammatable[i] = 255 - x;
      } else {
         gammatable[i] = x;
      }
      //Serial.println(gammatable[i]);
   }
   // servo setup
   sorterServo.attach(SERVO_PIN);
   sorterServo.setTimerWidth(14);
   sorterServo.setPeriodHertz(50);
   sorterServo.write(45);
   // sorterServo.write(30);
   WiFi.mode(WIFI_STA);
   // Init ESP-NOW
   if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
   }
   esp_now_register_recv_cb(OnDataRecv);

   previousMillisColour = millis();
   previousMillisServo = millis();
   previousMillisStage = millis();
} // end of setup

void loop() {
   long motorPosition = 0; // represented in encouder counts
   int potentiometer = 0; // raw ADC value from potentiometer
   noInterrupts();  
   motorPosition = encoder.position; // getting the current encoder position, placed in here so that interupts wont mess with the value                                                 
   interrupts();                                                               
   buttonDebounce();
   // Serial.println(sorterStage);
   // Default Rest Mode
   if (robotModeIndex == 0){
      scoopMotor.stop();
      // set leds to white
      setLedColor(100, 100, 100);
      robotModeIndex = 0;
      previousMillisStage = millis();
      previousMillisServoVibrate = millis();
      sorterStage = ROTATE_SCOOP;
      if (readColor()){
         // print the normalized rgb values 
         Serial.print("R: "); Serial.print(red/255.0);
         Serial.print(" G: "); Serial.print(green/255.0);
         Serial.print(" B: "); Serial.println(blue/255.0);
      }
   }

   // Operational Mode
   else{
      // Serial.println("operational mode");
      switch (sorterStage){
         case ROTATE_SCOOP:
            setLedColor(100, 0, 100); // set led to purple
            if (millis() - previousMillisStage > 1000){ // 1 Second delay before scoop starts to turn
               vibrateServo();
               if (scoopMotor.driveTo(PULSES_PER_REVOLUTION*2.75, motorPosition, PULSES_PER_REVOLUTION/16, toPWM(100))){
                  previousMillisStage = millis();
                  previousMillisServo = millis();
                  sorterStage = START_SORTER;
               }   
            }
            break;

         case START_SORTER:
            setLedColor(0,0,0); // set led to off
            if (readColor()){
               result = compareColor(green/255.0);
               int vibOffset;
               scoopVibrateToggle == false ? vibOffset = 0.05 : vibOffset = -0.05;
               if (scoopMotor.driveTo(PULSES_PER_REVOLUTION*(1.5+vibOffset), motorPosition, PULSES_PER_REVOLUTION/32, toPWM(100))){
                  scoopVibrateToggle == false ? scoopVibrateToggle = true : scoopVibrateToggle = false;
               }

               if (result == 'g'){
                  turnServo(0);
               }
               else{
                  turnServo(1);
               }
            }  
            if (millis() - previousMillisStage > 20000){ // Sorts for 10 seconds
               sorterStage = STOP;
            }
            break;

         case STOP:
            // Serial.println("STOP");
            setLedColor(100, 0, 100); // set led to purple
            if (scoopMotor.driveTo(0, motorPosition, PULSES_PER_REVOLUTION/16, toPWM(100))){ // Drive Scoop back to start
               sorterServo.write(45); // reset servo to middle position
               robotModeIndex = 0;
            }
            break;
      }   
   }        
} // end of main loop

bool readColor(){
   if (millis() - previousMillisColour > 60) {
      tcs.setInterrupt(false);  // turn on LED
      tcs.getRGB(&red, &green, &blue);
      tcs.setInterrupt(true);  // turn off LED
      previousMillisColour = millis();

      // Serial.print("R:\t"); Serial.print(int(red)); 
      // Serial.print("\tG:\t"); Serial.print(int(green)); 
      // Serial.println("\tB:\t"); Serial.print(int(blue));
      return true;
   }
   return false;
}

// converts a speed from 0 to 100 to a PWM value
unsigned char toPWM(float speed){
   unsigned char result = map(speed, 0, 100, MIN_PWM, MAX_PWM);
   return (result);
}
 
void turnServo(int choice){
   currentMillisServo = millis();
   if (servoToggle == true){
      previousMillisServo = millis();
      servoToggle = false;
   }
   switch (choice){
      case 0:
         if (currentMillisServo - previousMillisServo > SERVO_PAUSE_TIME){
            // Serial.println("Turning Servo");
            sorterServo.write(0);
            servoToggle = true;
         }
         else{
            vibrateServo();  
         }
         break;
      case 1:
         if (currentMillisServo - previousMillisServo > SERVO_PAUSE_TIME){
            // Serial.println("Turning Servo");
            sorterServo.write(90);
            servoToggle = true;
         }
         else{
            vibrateServo();
         }
         break;
   }
}

void vibrateServo(){   
   if (millis() - previousMillisServoVibrate > 100){
      sorterServo.write(43);
      if (millis() - previousMillisServoVibrate > 200){
         sorterServo.write(57);
         previousMillisServoVibrate = millis();
      }
   }

}

// converts rgb values to a hue
float toHue(float r, float g, float b){
   float hue = 0;
   hue = atan2(sqrt(3)*(g-b), 2*r-g-b);

   return hue;
}

char compareColor(float g){
   if (g > greenValue){
      return 'g';
   }
   else{
      return 'r';
   }
}


void setLedColor(int r, int g, int b){
   if (smartLED.getPixelColor(0) != smartLED.Color(g, r, b)){
      smartLED.setPixelColor(0, smartLED.Color(g, r, b));
      smartLED.show();
   }
}

// encoder interrupt service routine
// argument is pointer to an encoder structure, which is statically cast to a Encoder structure, allowing multiple
// instances of the encoderISR to be created (1 per encoder)
void ARDUINO_ISR_ATTR encoderISR(void* arg) {
   Encoder* s = static_cast<Encoder*>(arg);         // cast pointer to static structure
   
   int b = digitalRead(s->channalB);                   // read state of channel B
   if (b > 0) {                                     // high, leading channel A
      s->position--;                                      // decrease position
   }
   else {                                           // low, lagging channel A
      s->position++;                                      // increase position
   }
}

int mapToServo(int value){
   return map(value, 0, 180, 0, 16384);
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
