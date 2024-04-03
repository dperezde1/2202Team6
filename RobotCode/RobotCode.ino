// final commit

#define DEBUG_ENCODER_COUNT  1

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>

// Function declarations
void Indicator();                                                              // for mode/heartbeat on Smart LED
void scoopStop();
void scoopForward();
void scoopReverse();
// Port pin constants
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
// Scoop pin additions
#define SCOOP_MOTOR_A       39
#define SCOOP_MOTOR_B       40
#define SCOOP_ENCODER_A     5
#define SCOOP_ENCODER_B     6   
//Servo Pins
#define LEFT_SERVO          41    
#define RIGHT_SERVO         42    
#define SCOOP_MOTOR_SPEED 130


// Constants
const int cDisplayUpdate = 100;                                                // update interval for Smart LED in milliseconds
const int cPWMRes = 8;                                                         // bit resolution for PWM
const int cMinPWM = 150;                                                       // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;                                       // PWM value for maximum speed
const int cLeftAdjust = 0;                                                     // Amount to slow down left motor relative to right
const int cRightAdjust = 0;                                                    // Amount to slow down right motor relative to left

// Variables
boolean motorsEnabled = true;                                                  // motors enabled flag
boolean timeUp3sec = false;                                                    // 3 second timer elapsed flag
boolean timeUp2sec = false;                                                    // 2 second timer elapsed flag
boolean timeUp200msec = false;                                                 // 200 millisecond timer elapsed flag
unsigned char leftDriveSpeed;                                                  // motor drive speed (0-255)
unsigned char rightDriveSpeed;                                                 // motor drive speed (0-255)
unsigned char driveIndex;                                                      // state index for run mode
unsigned int modePBDebounce;                                                   // pushbutton debounce timer count
unsigned long timerCount3sec = 0;                                              // 3 second timer count in milliseconds
unsigned long timerCount2sec = 0;                                              // 2 second timer count in milliseconds
unsigned long timerCount200msec = 0;                                           // 200 millisecond timer count in milliseconds
unsigned long displayTime;                                                     // heartbeat LED update timer
unsigned long previousMicros;                                                  // last microsecond count
unsigned long currentMicros;                                                   // current microsecond count
const int cCountsRev = 1096;                                                   
const float circum = 13.8;                                                    // wheel circumference
const float cmPerPulse = circum/ (float)cCountsRev;                            // cm per pulse

// Scoop Variables
unsigned char scoopDriveSpeed;
// Define an array where each index corresponds to a driveIndex, and the value indicates the scoop direction (true for forward, false for reverse)
boolean scoopDirections[] = {true,true, true, true, true, true, true, true, true, true, false,false,false,false};

//Servo Variables

const int leftServoUp =400;                                               
const int leftServoDown = 1350;          //1350                               
const int rightServoUp = 2000;                                                 
const int rightServoDown = 1690;           //1690           

//Case 10 vars
unsigned long case10StartTime = 0; // Variable to store the start time of case 10
boolean case10DelayStarted = false; // Flag to indicate whether the delay has started

//Case 11
unsigned long case11StartTime = 0;
boolean case11DelayStarted = false;

//case 12
unsigned long case12StartTime = 0;
boolean case12DelayStarted = false;
int case12To11LoopCount = 0; // Tracks the number of loops from case 12 to case 11



// Make a square
const float targetPulses1 = (60 / cmPerPulse);                                // forward
const float targetPulses2 = (11.5 / cmPerPulse);                                // turn left
const float targetPulses3 = (30 / cmPerPulse);                                // forward
const float targetPulses4 = (11.7 / cmPerPulse);                                // turn left
const float targetPulses5 = (50 / cmPerPulse);                                // forward
const float targetPulses6 = (11.9 / cmPerPulse);                                // turn left
const float targetPulses7 = (30 / cmPerPulse);                                // forward
const float targetPulses8 = (12.7 / cmPerPulse);                                // turn left
const float targetPulses9 = (10 / cmPerPulse);                                // reverse for style


int currentPulsesLeft = 0;
int currentPulsesRight = 0;
int averagePulses = 0;

// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number 
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(SMART_LED_COUNT, SMART_LED, NEO_RGB + NEO_KHZ800);

// smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {5,15,30,45,60,75,90,105,120,135,150,165,180,195,210,225,240,255,
                                       240,225,210,195,180,165,150,135,120,105,90,75,60,45,30,15};

unsigned int  robotModeIndex = 0;                                              // robot operational state                              
unsigned int  modeIndicator[6] = {                                             // colours for different modes
   SmartLEDs.Color(255,0,0),                                                   //   red - stop
   SmartLEDs.Color(0,255,0),                                                   //   green - run
   SmartLEDs.Color(0,0,255),                                                   //   blue - empty case
   SmartLEDs.Color(255,255,0),                                                 //   yellow - empty case
   SmartLEDs.Color(0,255,255),                                                 //   cyan - empty case
   SmartLEDs.Color(255,0,255)                                                  //   magenta - empty case
};                                                                            

// Motor, encoder, and IR objects (classes defined in MSE2202_Lib)
Motion Bot = Motion();                                                         // Instance of Motion for motor control
Encoders LeftEncoder = Encoders();                                             // Instance of Encoders for left encoder data
Encoders RightEncoder = Encoders();                                            // Instance of Encoders for right encoder data




void setup() {
  #if defined DEBUG_ENCODER_COUNT
   Serial.begin(115200);
#endif
   
  // Set up motors and encoders
   Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B); // set up motors as Drive 1
   LeftEncoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B, &Bot.iLeftMotorRunning ); // set up left encoder
   RightEncoder.Begin(ENCODER_RIGHT_A, ENCODER_RIGHT_B, &Bot.iRightMotorRunning ); // set up right encoder
   Bot.servoBegin("S1", LEFT_SERVO);                                           // set up claw servo
   Bot.servoBegin("S2", RIGHT_SERVO);                                       // set up shoulder servo
 


  //Set up scoop
  pinMode(SCOOP_MOTOR_A, OUTPUT);
  pinMode(SCOOP_MOTOR_B, OUTPUT);





   // Set up SmartLED
   SmartLEDs.begin();                                                          // initialize smart LEDs object (REQUIRED)
   SmartLEDs.clear();                                                          // clear pixel
   SmartLEDs.setPixelColor(0,SmartLEDs.Color(0,0,0));                          // set pixel colors to 'off'
   SmartLEDs.show();                                                           // send the updated pixel colors to the hardware

   pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);                                 // set up motor enable switch with internal pullup
   pinMode(MODE_BUTTON, INPUT_PULLUP);                                         // Set up mode pushbutton
   modePBDebounce = 0;                                                         // reset debounce timer count

}

void loop() {
   long pos[] = {0, 0};                                                        // current motor positions
   int pot = 0;                                                                // raw ADC value from pot

   // Timer setup
   currentMicros = micros();                                                   // get current time in microseconds
   if ((currentMicros - previousMicros) >= 1000) {                             // enter when 1 s has elapsed
      previousMicros = currentMicros;                                          // record current time in microseconds

      // 3 second timer, counts 3000 milliseconds
      timerCount3sec = timerCount3sec + 1;                                     // increment 3 second timer count
      if (timerCount3sec > 3000) {                                             // if 3 seconds have elapsed
         timerCount3sec = 0;                                                   // reset 3 second timer count
         timeUp3sec = true;                                                    // indicate that 3 seconds have elapsed
      }
   
      // 2 second timer, counts 2000 milliseconds
      timerCount2sec = timerCount2sec + 1;                                     // increment 2 second timer count
      if (timerCount2sec > 2000) {                                             // if 2 seconds have elapsed
         timerCount2sec = 0;                                                   // reset 2 second timer count
         timeUp2sec = true;                                                    // indicate that 2 seconds have elapsed
      }
   
      // 200 millisecond timer, counts 200 milliseconds
      timerCount200msec = timerCount200msec + 1;                               // Increment 200 millisecond timer count
      if(timerCount200msec > 200)                                              // If 200 milliseconds have elapsed
      {
         timerCount200msec = 0;                                                // Reset 200 millisecond timer count
         timeUp200msec = true;                                                 // Indicate that 200 milliseconds have elapsed
      }

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
               robotModeIndex++;                                               // switch to next mode
               robotModeIndex = robotModeIndex & 7;                            // keep mode index between 0 and 7
               timerCount3sec = 0;                                             // reset 3 second timer count
               timeUp3sec = false;                                             // reset 3 second timer
            }
         }
      }
  
      // check if drive motors should be powered
      motorsEnabled = !digitalRead(MOTOR_ENABLE_SWITCH);                       // if SW1-1 is on (low signal), then motors are enabled

      // modes 
      // 0 = Default after power up/reset. Robot is stopped.
      // 1 = Press mode button once to enter.        Run robot
      // 2 = Press mode button twice to enter.       Blue
      // 3 = Press mode button three times to enter. Yellow
      // 4 = Press mode button four times to enter.  Light Blue
      // 5 = Press mode button five times to enter.  Add your code to do something 
      // 6 = Press mode button six times to enter.   Add your code to do something 

    switch(robotModeIndex) {
         case 0: // Robot stopped
            Bot.Stop("D1");    
            LeftEncoder.clearEncoder();                                        // clear encoder counts
            RightEncoder.clearEncoder();
            driveIndex = 0;                                                    // reset drive index
            scoopStop();
            //timeUp2sec = false;                                                // reset 2 second timer
            break;

         case 1: // Run robot
            if (timeUp3sec) {                                                  // pause for 3 sec before running case 1 code
               // Read pot to update drive motor speed
               pot = analogRead(POT_R1);
               leftDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM) - cLeftAdjust;
               rightDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM) - cRightAdjust;
              #ifdef DEBUG_DRIVE_SPEED 
               Serial.print(F(" Left Drive Speed: Pot R1 = "));
               Serial.print(pot);
               Serial.print(F(", mapped = "));
               Serial.println(leftDriveSpeed);
              #endif
              #ifdef DEBUG_ENCODER_COUNT
               if (timeUp200msec) {
                  timeUp200msec = false;                                       // reset 200 ms timer
                  LeftEncoder.getEncoderRawCount();                            // read left encoder count 
                  RightEncoder.getEncoderRawCount();                           // read right encoder count
                  Serial.print(F("Left Encoder count = "));
                  Serial.print(LeftEncoder.lRawEncoderCount);
                  Serial.print(F("  Right Encoder count = "));
                  Serial.print(RightEncoder.lRawEncoderCount);
                  Serial.print("\n");
               }
      #endif
               if (motorsEnabled) {  //run motors only if enabled
               /* if (scoopDirections[driveIndex]) {
                  scoopForward();
                } else {
                  scoopReverse();
                }*/
                     switch(driveIndex) {                                      // cycle through drive states
                        case 0: // Stop
                          scoopStop();
                          Bot.ToPosition("S1", leftServoUp);
                          Bot.ToPosition("S2", rightServoUp);
                          LeftEncoder.clearEncoder();
                          RightEncoder.clearEncoder();
                          Bot.Stop("D1");                                     // drive ID
                          driveIndex++;                                       // next state: drive forward
                          break;

                        case 1: // Drive forward
                          scoopForward();
                          LeftEncoder.getEncoderRawCount();                            // read left encoder count 
                          RightEncoder.getEncoderRawCount(); 
                          // determine the average pulses by averaging both motor encoder counts
                          currentPulsesLeft = abs(LeftEncoder.lRawEncoderCount);
                          currentPulsesRight = abs(RightEncoder.lRawEncoderCount);
                          averagePulses = (currentPulsesLeft + currentPulsesRight)/2;
                          
                          Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed
                          Serial.print("Average Pulses = ");
                          Serial.print(averagePulses);
                          Serial.print("\n");
                          


                          if (averagePulses >= targetPulses1) {
                            LeftEncoder.clearEncoder();
                            RightEncoder.clearEncoder();
                            driveIndex++;                                       // next state: turn right
                          }
                          
                          break;

                        case 2: // Turn left
                          
                          LeftEncoder.getEncoderRawCount();                            // read left encoder count 
                          RightEncoder.getEncoderRawCount(); 
                          currentPulsesLeft = abs(LeftEncoder.lRawEncoderCount);
                          currentPulsesRight = abs(RightEncoder.lRawEncoderCount);
                          averagePulses = (currentPulsesLeft + currentPulsesRight)/2;

                          Bot.Left("D1", 200, 200);    // drive ID, left speed, right speed
                          if (averagePulses >= targetPulses2){
                            LeftEncoder.clearEncoder();
                            RightEncoder.clearEncoder();
                            driveIndex++;                                     // next state: forward
                          }
                                                               
                          break;
                        case 3: // Drive forward
                          LeftEncoder.getEncoderRawCount();                            // read left encoder count 
                          RightEncoder.getEncoderRawCount(); 
                          // determine the average pulses by averaging both motor encoder counts
                          currentPulsesLeft = abs(LeftEncoder.lRawEncoderCount);
                          currentPulsesRight = abs(RightEncoder.lRawEncoderCount);
                          averagePulses = (currentPulsesLeft + currentPulsesRight)/2;
                          
                          Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed
                          Serial.print("Average Pulses = ");
                          Serial.print(averagePulses);
                          Serial.print("\n");


                          if (averagePulses >= targetPulses3) {
                            LeftEncoder.clearEncoder();
                            RightEncoder.clearEncoder();
                            driveIndex++;                                       // next state: turn right
                          }
                          
                          break;

                        case 4: // Turn left
                          
                          LeftEncoder.getEncoderRawCount();                            // read left encoder count 
                          RightEncoder.getEncoderRawCount(); 
                          currentPulsesLeft = abs(LeftEncoder.lRawEncoderCount);
                          currentPulsesRight = abs(RightEncoder.lRawEncoderCount);
                          averagePulses = (currentPulsesLeft + currentPulsesRight)/2;

                          Bot.Left("D1", 200, 200);    // drive ID, left speed, right speed
                          if (averagePulses >= targetPulses2){
                            LeftEncoder.clearEncoder();
                            RightEncoder.clearEncoder();
                            driveIndex++;                                     // next state: forward
                          }
                                                               
                          break;
                        case 5: // Drive forward
                          LeftEncoder.getEncoderRawCount();                            // read left encoder count 
                          RightEncoder.getEncoderRawCount(); 
                          // determine the average pulses by averaging both motor encoder counts
                          currentPulsesLeft = abs(LeftEncoder.lRawEncoderCount);
                          currentPulsesRight = abs(RightEncoder.lRawEncoderCount);
                          averagePulses = (currentPulsesLeft + currentPulsesRight)/2;
                          
                          Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed
                          Serial.print("Average Pulses = ");
                          Serial.print(averagePulses);
                          Serial.print("\n");


                          if (averagePulses >= targetPulses5) {
                            LeftEncoder.clearEncoder();
                            RightEncoder.clearEncoder();
                            driveIndex++;                                       // next state: turn right
                          }
                          
                          break;

                        case 6: // Turn left
                          
                          LeftEncoder.getEncoderRawCount();                            // read left encoder count 
                          RightEncoder.getEncoderRawCount(); 
                          currentPulsesLeft = abs(LeftEncoder.lRawEncoderCount);
                          currentPulsesRight = abs(RightEncoder.lRawEncoderCount);
                          averagePulses = (currentPulsesLeft + currentPulsesRight)/2;

                          Bot.Left("D1", 200, 200);    // drive ID, left speed, right speed
                          if (averagePulses >= targetPulses6){
                            LeftEncoder.clearEncoder();
                            RightEncoder.clearEncoder();
                            driveIndex++;                                     // next state: forward
                          }
                                                               
                          break;
                        case 7: // Drive forward
                          LeftEncoder.getEncoderRawCount();                            // read left encoder count 
                          RightEncoder.getEncoderRawCount(); 
                          // determine the average pulses by averaging both motor encoder counts
                          currentPulsesLeft = abs(LeftEncoder.lRawEncoderCount);
                          currentPulsesRight = abs(RightEncoder.lRawEncoderCount);
                          averagePulses = (currentPulsesLeft + currentPulsesRight)/2;
                          
                          Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed
                          Serial.print("Average Pulses = ");
                          Serial.print(averagePulses);
                          Serial.print("\n");


                          if (averagePulses >= targetPulses7) {
                            LeftEncoder.clearEncoder();
                            RightEncoder.clearEncoder();
                            driveIndex++;                                       // next state: stop
                          }
                          
                          break;
      
                        case 8: // Turn left
                          
                          LeftEncoder.getEncoderRawCount();                            // read left encoder count 
                          RightEncoder.getEncoderRawCount(); 
                          currentPulsesLeft = abs(LeftEncoder.lRawEncoderCount);
                          currentPulsesRight = abs(RightEncoder.lRawEncoderCount);
                          averagePulses = (currentPulsesLeft + currentPulsesRight)/2;

                          Bot.Left("D1", 200, 200);    // drive ID, left speed, right speed
                          if (averagePulses >= targetPulses8){
                            LeftEncoder.clearEncoder();
                            RightEncoder.clearEncoder();
                            driveIndex++;                                     // next state: forward
                          }
                                                               
                          break;
                        
                        case 9: // Reverse for style
                          LeftEncoder.getEncoderRawCount();                            // read left encoder count 
                          RightEncoder.getEncoderRawCount(); 
                          // determine the average pulses by averaging both motor encoder counts
                          currentPulsesLeft = abs(LeftEncoder.lRawEncoderCount);
                          currentPulsesRight = abs(RightEncoder.lRawEncoderCount);
                          averagePulses = (currentPulsesLeft + currentPulsesRight)/2;
                          
                          Bot.Reverse("D1", 200, 200); // drive ID, left speed, right speed
                          Serial.print("Average Pulses = ");
                          Serial.print(averagePulses);
                          Serial.print("\n");


                          if (averagePulses >= targetPulses9) {
                            LeftEncoder.clearEncoder();
                            RightEncoder.clearEncoder();
                            driveIndex++;                                       // next state: turn right
                          }
                          
                          break;
                        case 10:
                            if (!case10DelayStarted) {
                              Serial.print("starting case 10");
                              Bot.Stop("D1");
                              Bot.ToPosition("S1", leftServoDown);                             
                              Bot.ToPosition("S2", rightServoDown);
                              scoopReverse();
                              Serial.print("end of servo movement");
                              LeftEncoder.clearEncoder();
                              RightEncoder.clearEncoder();

                              case10StartTime = millis(); // Save the start time for the delay
                              case10DelayStarted = true; // Mark the delay as started
                            }

                            if (millis() - case10StartTime >= 3000) { // Check if 3 seconds have passed
                              // Actions to be performed after the 3-second delay
                            
                              case10DelayStarted = false;
                              driveIndex++; // Reset the delay flag for the next time this case runs
                            }
                            break;
                        case 11:
                            if (!case11DelayStarted) {
                              Serial.println("Starting case 11: Stopping scoop and moving servos.");
                              scoopStop(); // Stop the scoop
                              Bot.ToPosition("S1", leftServoUp); // Move servos to up position
                              Bot.ToPosition("S2", rightServoUp);
                              
                              case11StartTime = millis();
                              case11DelayStarted = true;
                            }

                            // Check if 500 milliseconds have passed to move servos back down
                            if (millis() - case11StartTime >= 300 && case11DelayStarted) {
                              Bot.ToPosition("S1", leftServoDown); // Move servos back to down position
                              Bot.ToPosition("S2", rightServoDown);

                              Serial.println("Case 11 complete: Servos moved back down.");
                              case11DelayStarted = false; // Reset for next time case 11 is entered
                              scoopReverse();
                              driveIndex++; // Optionally, reset to default case or navigate to another case
                            }
                            break;
                        case 12:
                            if (!case12DelayStarted) {
                              Serial.println("Starting case 12: Delay for 3 seconds.");
                              case12StartTime = millis(); // Record start time
                              case12DelayStarted = true;
                            }

                            if (millis() - case12StartTime >= 3000) { // Check if 3 seconds have passed
                              case12To11LoopCount++; // Increment loop count
                              case12DelayStarted = false; // Reset delay flag for next iteration

                              if (case12To11LoopCount < 3) {
                                Serial.println("Repeating case 11.");
                                driveIndex = 11; // Go back to case 11
                              } else {
                                Serial.println("Completed 3 iterations, going to robotModeIndex = 0.");
                                robotModeIndex = 0; // Go to default case after 3 iterations
                                case12To11LoopCount = 0; // Reset loop count for next time
                              }
                            }
                            break;

                     }
                  
               }
            }
            else {                                                             // stop when motors are disabled
               Bot.Stop("D1");  
            }
            break;
          scoopStop();
          robotModeIndex = 0;           // to red mode
            
            break;

      }

 // Update brightness of heartbeat display on SmartLED
      displayTime++;                                                          // count milliseconds
      if (displayTime > cDisplayUpdate) {                                     // when display update period has passed
         displayTime = 0;                                                     // reset display counter
         LEDBrightnessIndex++;                                                // shift to next brightness level
         if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) {              // if all defined levels have been used
            LEDBrightnessIndex = 0;                                           // reset to starting brightness
         }
         SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);    // set brightness of heartbeat LED
         Indicator();                                                         // update LED
      }
   }
}   

// Set colour of Smart LED depending on robot mode (and update brightness)
void Indicator() {
  SmartLEDs.setPixelColor(0, modeIndicator[robotModeIndex]);                  // set pixel colors to = mode 
  SmartLEDs.show();                                                           // send the updated pixel colors to the hardware
}

//Scoop functions
void scoopForward() {
  analogWrite(SCOOP_MOTOR_A, SCOOP_MOTOR_SPEED); // Set the speed for forward direction
  digitalWrite(SCOOP_MOTOR_B, LOW);              // Ensure the other pin is off
}

void scoopReverse() {
  digitalWrite(SCOOP_MOTOR_A, LOW);              // Ensure one pin is off
  digitalWrite(SCOOP_MOTOR_B, HIGH); // Set the speed for reverse direction
}

void scoopStop() {
  // Stop the motor by writing a low signal to both control pins
  digitalWrite(SCOOP_MOTOR_A, LOW);
  digitalWrite(SCOOP_MOTOR_B, LOW);
}

