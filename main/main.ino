#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TCS34725.h>
#include <Wire.h>
#include <ESP32Servo.h>


#define LEFT_MOTOR_A        35                                                 // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B        36                                                 // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A       37                                                 // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B       38                                                 // GPIO38 pin 31 (J38) Motor 2 B
#define ENCODER_LEFT_A      15                                                 // left encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_LEFT_B      16                                                 // left encoder B signal is connected to pin 8 GPIO16 (J16)
#define MODE_BUTTON         0                                                  // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH 3                                                  // DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
#define POT_R1              1                                                  // when DIP Switch S1-3 is on, Analog AD0 (pin 39) GPIO1 is connected to Poteniometer R1
#define SMART_LED           21                                                 // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT     1                                                  // number of SMART LEDs in use
// set to false if using a common cathode LED
#define commonAnode true
#define SWITCH_PIN          46

unsigned int robotModeIndex = 0;                                              // robot operational state 
unsigned int debounceTimer;         

unsigned int previousMillisColour;
unsigned int currentMillisColour;
unsigned int previousMillisServo;
unsigned int currentMillisServo;


const int PWM_Resolution = 8;                                                  // bit resolution
const int MIN_PWM = 150;                                                       
const int MAX_PWM = pow(2, PWM_Resolution) - 1;    
const int PULSES_PER_REVOLUTION = 1096;                                        // encoder pulses per motor revolution
                                           
byte gammatable[256];
float red, green, blue; // for color sensor

const int servoPin          = 41;                 // GPIO pin for servo motor

Adafruit_NeoPixel smartLED = Adafruit_NeoPixel(SMART_LED_COUNT, SMART_LED, NEO_GRB + NEO_KHZ800);  

// Motor, encoder, and IR objects (classes defined in MSE2202_Lib)
Motion Bot = Motion();                                                         
Encoders LeftEncoder = Encoders();  

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

Servo sorterServo;
int servoPos = 0;

long targetPos = 0;

char result = 'w';

bool servoToggle = false;

bool roboMode = false;

void setup() {
   smartLED.begin();                                                           
   smartLED.show(); // Initialize all pixels to 'off'

   pinMode(SWITCH_PIN, INPUT_PULLUP);

   sorterServo.attach(servoPin);

   Wire.begin(47,48);

   if (tcs.begin()) {
   //Serial.println("Found sensor");
   } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
   }
   
  // Set up motors and encoders
   Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B); 
   LeftEncoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B, &Bot.iLeftMotorRunning ); 
   pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);
   pinMode(MODE_BUTTON, INPUT_PULLUP);


   debounceTimer = 0;
   // thanks PhilB for this gamma table!
   // it helps convert RGB colors to what humans see
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

   previousMillisColour = millis();
   previousMillisServo = millis();
}

void loop() {
   long motorPosition[] = {0, 0}; // represented in encouder counts
   int potentiometer = 0; // raw ADC value from potentiometer

   digitalRead(SWITCH_PIN) == HIGH ? roboMode = true : roboMode = false;



   noInterrupts();                                                             
   LeftEncoder.getEncoderRawCount();
   motorPosition[0] = LeftEncoder.lRawEncoderCount;                                                 
   interrupts();                                                               

   buttonDebounce();

   if (robotModeIndex == 0){
   
      Bot.Stop("D1");
      LeftEncoder.clearEncoder(); 
      Serial.println("Robot Stopped");                                       
      // set leds to red
      setLedColor(100, 100, 100);
      robotModeIndex = 0;
   }
   else{
      // set led to purple
      setLedColor(100, 0, 100);

      if (roboMode == true){
         // potentiometer = analogRead(POT_R1); // read potentiometer value
         // targetPos = map(potentiometer, 0, 4095, 0, 100); // map potentiometer value to motor position
         // if (targetPos > 100){
         //    Bot.Forward("D1", toPWM(100), toPWM(100));
         // }
         // else{
         //    Bot.Stop("D1");
         // }
         Bot.Forward("D1", toPWM(100), toPWM(100));

      }
      else{
         if (readColor()){
            // set leds to color
            result = maxColor();
            if (result == 'g'){
               turnServo(0);

            }
            else{
               turnServo(1);
            }
         }        
      }      
   }  
      
}


bool readColor(){

   currentMillisColour = millis();
   if (currentMillisColour - previousMillisColour > 60) {

      tcs.setInterrupt(false);  // turn on LED


      tcs.getRGB(&red, &green, &blue);

      tcs.setInterrupt(true);  // turn off LED
      previousMillisColour = currentMillisColour;

      // Serial.print("R:\t"); Serial.print(int(red)); 
      // Serial.print("\tG:\t"); Serial.print(int(green)); 
      // Serial.println("\tB:\t"); Serial.print(int(blue));
      return true;
   }
   return false;

}


void driveTo(long distance, long motorPosition){
   if(motorPosition < distance){ 
      Bot.Forward("D1", toPWM(100), 0);  
   }
   if (motorPosition > distance){
      Bot.Reverse("D1", toPWM(100), 0);
   }
}

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
         if (currentMillisServo - previousMillisServo > 3000){
            sorterServo.write(0);
            servoToggle = true;
         }
         else{
            sorterServo.write(45);
         }
         break;
      case 1:
         if (currentMillisServo - previousMillisServo > 3000){
            sorterServo.write(90);
            servoToggle = true;
         }
         else{
            sorterServo.write(45);
         }
         break;
   }





}


char maxColor(){
   if (red > green && red > blue){
      red = 255;
      green = 0;
      blue = 0;
      return('r');
   }
   else if (green > red && green > blue){
      red = 0;
      green = 255;
      blue = 0;
      return('g');
   }
   else if (blue > red && blue > green){
      red = 0;
      green = 0;
      blue = 255;
      return('b');
   }
   else{
      red = 255;
      green = 255;
      blue = 255;
      return('w');
   }  

}

void setLedColor(int r, int g, int b){
   if (smartLED.getPixelColor(0) != smartLED.Color(g, r, b)){
      smartLED.setPixelColor(0, smartLED.Color(g, r, b));
      smartLED.show();
   }
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

