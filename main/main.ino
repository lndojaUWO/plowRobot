#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>


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

unsigned char driveIndex;                                                      // state index for run mode
unsigned int modePBDebounce;                                                   // pushbutton debounce timer count
unsigned long previousMillis;                                                  // last microsecond count
unsigned long currentMillis;                                                   // current microsecond count

const int cPWMRes = 8;                                                         // bit resolution for PWM
const int cMinPWM = 150;                                                       // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;    
const int cCountsRev = 1096;                                                   // encoder pulses per motor revolution
unsigned char leftDriveSpeed;                                                  // motor drive speed (0-255)
unsigned char rightDriveSpeed;                                                 // motor drive speed (0-255)

const int WHEEL_DIAMETER = 5;
unsigned int  robotModeIndex = 0;                                              // robot operational state                              

// Motor, encoder, and IR objects (classes defined in MSE2202_Lib)
Motion Bot = Motion();                                                         // Instance of Motion for motor control
Encoders LeftEncoder = Encoders();                                             // Instance of Encoders for left encoder data
Encoders RightEncoder = Encoders();                                            // Instance of Encoders for right encoder data

void setup() {
#if defined DEBUG_DRIVE_SPEED || DEBUG_ENCODER_COUNT
   Serial.begin(115200);
#endif
   
  // Set up motors and encoders
   Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B); // set up motors as Drive 1
   LeftEncoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B, &Bot.iLeftMotorRunning ); // set up left encoder
   RightEncoder.Begin(ENCODER_RIGHT_A, ENCODER_RIGHT_B, &Bot.iRightMotorRunning ); // set up right encoder

   pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);                                 // set up motor enable switch with internal pullup
   pinMode(MODE_BUTTON, INPUT_PULLUP);                                         // Set up mode pushbutton
   modePBDebounce = 0;                                                         // reset debounce timer count
}

void loop() {
   long pos[] = {0, 0};                                                        // current motor positions
   int pot = 0;                                                                // raw ADC value from pot

   noInterrupts();                                                             // disable interrupts temporarily while reading
   LeftEncoder.getEncoderRawCount();
   RightEncoder.getEncoderRawCount();
   pos[0] = LeftEncoder.lRawEncoderCount;                                                 // read and store current motor position
   pos[1] = RightEncoder.lRawEncoderCount;
   interrupts();                                                               // turn interrupts back on

   currentMillis = millis();
   buttonDebounce();

    switch(robotModeIndex) {
        case 0:
            Bot.Stop("D1");
            LeftEncoder.clearEncoder();                                        // clear encoder counts
            RightEncoder.clearEncoder();
            break;
        case 1:
            pot = analogRead(POT_R1);
            leftDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM);
            rightDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM);
            if(pos[0] < getCM(20)){ 
                        Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed  
                     }
                     else{
                        Bot.Stop("D1");
                        LeftEncoder.clearEncoder();
                        RightEncoder.clearEncoder();
                        robotModeIndex = 0;
                     }          

    }

}


void buttonDebounce(){
    // Mode pushbutton debounce and toggle
   if (!digitalRead(MODE_BUTTON)) {                                         // if pushbutton GPIO goes LOW (nominal push)
      // Start debounce
      if (modePBDebounce <= 25) {                                           // 25 millisecond debounce time
         modePBDebounce = modePBDebounce + 1;                               // increment debounce timer count
         if (modePBDebounce > 25) {                                         // if held for at least 25 mS
            modePBDebounce = 1000;                                          // change debounce timer count to 1 second
         }
      }
      if (modePBDebounce >= 1000) {                                         // maintain 1 second timer count until release
         modePBDebounce = 1000;
      }
   }
   else {                                                                   // pushbutton GPIO goes HIGH (nominal release)
      if(modePBDebounce <= 26) {                                            // if release occurs within debounce interval
         modePBDebounce = 0;                                                // reset debounce timer count
      }
      else {
         modePBDebounce = modePBDebounce + 1;                               // increment debounce timer count
         if(modePBDebounce >= 1025) {                                       // if pushbutton was released for 25 mS
            modePBDebounce = 0;                                             // reset debounce timer count
            previousMillis = millis();
            robotModeIndex++;                                               // switch to next mode
            robotModeIndex = robotModeIndex & 7;                            // keep mode index between 0 and 7
         }
      }
   }
}

long getCM(long inCM){ // input distance in cm and it returns the correct encoder value
   return(inCM * cCountsRev / (WHEEL_DIAMETER * 3.14159));
}
