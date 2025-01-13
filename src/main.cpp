//  Program to control my marshmallow roasting robot.  RF transmitter (see rhReliableXmit.ino for that code) used to 
//  drive it and control the slide arm.
// Transmitter right joystick - forward/backward/turn-left/turn-right.   Joystick pushButton-for-headlights on/off
// Transmitter left joystick - slide-out/slide-back/slide-up/slide-down.  Joystick pushButton-for-rotisserie on/off
//
//  dlf 12/12/2024

#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include <RHReliableDatagram.h>
#include <RH_NRF24.h>

int DEBUG = 0;  // 1 for debug, 0 for quiet

// Prototypes
void go_Forward();  //Set direction to Forward
void go_Left(); //Set direction to left
void spin_ccw();  //Set direction to rotate counter clockwise
void go_Right();  //Set direction to right
void spin_cw();  //Set direction to rotate in place clockwise
void go_Back();  //Set direction to
void left_stop(); // left wheel stop
void left_back(); // left wheel backward turn
void left_forward(); // left wheel forward turn
void right_stop(); // right wheel stop
void right_back(); // right wheel backward turn
void right_forward();  //right wheel forward turn
void allStop();    //Stop the robot
void set_MotorSpeed(int left,int right);
void updateSlidePulleyCount();
void extendSlide(); 
void retractSlide(); 
void stopSlide();
void raiseSlide(); 
void lowerSlide(); 
void homeTheSlide();
void sendMessageToClient(char * messageBuffer);
boolean checkSwitch(int pin, boolean doDebounceCheck);
boolean checkSwitchA6A7(int pin, boolean doDebounceCheck);

// For the nrf24 xmit/receive
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

// SPI pins are d13 (SCK), d12 (MISO), d11 (MOSI)

#define slideLimitSwitchPin A6

// Motion/motor control 
#define speedPinR 5              // Right Front WHEEL PWM pin D45 connect front MODEL-X ENA 
#define speedPinL 6              // Left Front WHEEL PWM pin D7 connect front MODEL-X ENB
#define rightMotorDirPin1 7      // Right Motor direction pin 1 to Front MODEL-X IN1  (K1)
#define rightMotorDirPin2 8      // Right Motor direction pin 2 to Front MODEL-X IN2   (K1)                                 
#define leftMotorDirPin1  9      // Left  Motor direction pin 1 to Front MODEL-X IN3 (  K3)
#define leftMotorDirPin2  10     // Left  Motor direction pin 2 to Front MODEL-X IN4 (  K3)

#define rotatorDirPin1 A4  // The motor that rotates the marshmallow rod.  No encoder or speed control on this motor

#define headlightControl A5           // High to turn on the headlights

#define slideMotorDirPin1 A2          // The motor that extends/retracts the slide
#define slideMotorDirPin2 A3          // The motor that extends/retracts the slide
#define slideMotorHallPin 2           // Interrupt pin for the Hall effect sensor to track slide motor rotation

#define linearActuatorDirPin1 3  // The linear actuator that raises/lowers the arm
#define linearActuatorDirPin2 4 

#define FULL_SPEED   255   //Full speed
#define HALF_SPEED   127   //Half speed 

// Joystick constants
#define JOY_MARGIN 20   // Don't respond to joystick unless it is moved this much (count is the 0-1023 AtoD range)


// Globals
int  SWITCH_PRESSED = 0;
int  MAGNET_SENSED = 0;
int RIGHT_MOTOR_SPEED_ADJ = 0;          // In case the left/right motors don't run the same speed (we are not using encoders... ran out of IO's!)
int LEFT_MOTOR_SPEED_ADJ = -7;
volatile int slideMotorCounter = 0;  // Keeps track of how far the slides have moved
volatile boolean interruptSync = false; //Set true right after an interrupt, clear when incrementing/decrementing the slideMotorCounter.  
int MAX_SLIDE_MOTOR_COUNT = 16;      // Slide extend limit.  Max number of half turns of the slide motor before stopping.
int MIN_SLIDE_MOTOR_COUNT = 1;       // Slide retract limit.  Min number of half turns of the slide motor before stopping.

// Radio variables
// Six character identifier string for the radio node
const byte address[6]="00001";
const byte radioChannel = 120;  // 1-125

// Data packet that will be sent from the transmitter
struct DataStructure {
   byte joyAddress;      // Which joystick
   int joyX;
   int joyY;
   int joyXCenter;       // Set to the initial center position of the joystick
   int joyYCenter;       // Set to the initial center position of the joystick
   byte joySwitch;
};
DataStructure packet;

// State vars
boolean newPacketAvailable = false;  // State used later so we don't evaluate packets unless there is a new one
boolean rotatorMotorOn = false;      // Indicates if marshmallow rotator motor is on or off
boolean headlightsOn = false;  // Headlight on/off state 
boolean slideExtending = false;  // Indicate if the slide is moving
boolean slideRetracting = false;

// Buffer to use for sending messages back to the transmitter
uint8_t messageBuffer[24];  // Character buffer for text sent back to us


// ####################
// Instantiate Objects
// ####################

// Radio module
RH_NRF24 driver(A0,A1);  // A0 is CE,  A1 is CSN

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);


//#######################################################
//#######################################################
// Functions
//#######################################################
//#######################################################

//##################################################
// Interrupt handler for the slide motor hall sensor
//##################################################
void updateSlidePulleyCount() {
   if(slideExtending) {
      slideMotorCounter++;  // Slide extending
   } else if(slideRetracting) {
      slideMotorCounter--;  // Slide extending
   }
   interruptSync = true;  // Tells the main loop that an interrupt happened so we can stop the pulley synchronously
}

//###############################################################################
// Retract the slide until we hit the limit switch, then reset the motor counter.
//###############################################################################
void homeTheSlide() {
      // first extend the slide a tick in case it's sitting on the limit switch
      // NOTE: Need to be sure the slide is not fully extended -  Otherwise this will jam it against the end stop.
      // I currently have no way of telling that the slide is fully extended during power-up... A limit switch
      // would melt over the campfire!
      interruptSync=false;
      extendSlide();  // Get it moving
      int curCount = slideMotorCounter;
      while(slideMotorCounter <= curCount + 1) {
      }
      stopSlide();

       // Move slides back to home
      if(DEBUG) {
         Serial.println(F("Moving slide to home position"));
      }
      retractSlide();

      // Wait until the limit switch tripped
      // No debounce.  Just want to see the first edge as quickly as possible.
      while(checkSwitchA6A7(slideLimitSwitchPin,false) != SWITCH_PRESSED) {
      }
      if(DEBUG) {
         Serial.println(F("Found limit switch"));
      }
      stopSlide();
      slideMotorCounter = 0;

      // Now advance slide to the minimum motor count position (we don't want the slide sitting at it's lower limit)
      extendSlide();  // Get it moving
      stopSlide();    // Stop at the next interrupt
}

//##############################
// Robot wheel movements
//##############################
void go_Forward()  //Set direction to Forward
{
   right_forward();
   left_forward();
}
void go_Left()  //Set direction to left
{
   right_forward();
   left_stop();
}
void go_Right()  //Set direction to right
{
   right_stop();
   left_forward();
}
void spin_cw()  //Set direction to rotate in place clockwise
{
   right_back();
   left_forward();
}
void spin_ccw()  //Set direction to rotate counter clockwise
{
   right_forward();
   left_back();
}
void go_Back()  //Set direction to
{
   right_back();
   left_back();
}

//Stop the robot 
void allStop() { 
   digitalWrite(rightMotorDirPin1, LOW);
   digitalWrite(rightMotorDirPin2,LOW);
   digitalWrite(leftMotorDirPin1,LOW);
   digitalWrite(leftMotorDirPin2,LOW);
   digitalWrite(linearActuatorDirPin1,LOW);
   digitalWrite(linearActuatorDirPin2,LOW);
   set_MotorSpeed(0,0);
   stopSlide();
}

//right wheel forward turn
void right_forward()  {
   digitalWrite(rightMotorDirPin1,LOW);
   digitalWrite(rightMotorDirPin2,HIGH); 
}
// right wheel backward turn
void right_back() {
   digitalWrite(rightMotorDirPin1,HIGH);
   digitalWrite(rightMotorDirPin2,LOW); 
}
// left wheel forward turn
void left_forward() {
   digitalWrite(leftMotorDirPin1,LOW);
   digitalWrite(leftMotorDirPin2,HIGH);
}
// left wheel backward turn
void left_back() {
   digitalWrite(leftMotorDirPin1,HIGH);
   digitalWrite(leftMotorDirPin2,LOW);
}
// right wheel stop
void right_stop() {
   digitalWrite(rightMotorDirPin1,LOW);
   digitalWrite(rightMotorDirPin2,LOW);
}
// left wheel stop
void left_stop() {
   digitalWrite(leftMotorDirPin1,LOW);
   digitalWrite(leftMotorDirPin2,LOW);
}
/*set motor speed-  This is what actually starts the motors moving */
void set_MotorSpeed(int left,int right) {
   left += LEFT_MOTOR_SPEED_ADJ;
   right += RIGHT_MOTOR_SPEED_ADJ;
   analogWrite(speedPinL,left); 
   analogWrite(speedPinR,right); 
}

//##############################
// Robot arm movements
//##############################
void extendSlide() {
   slideExtending = true;
   digitalWrite(slideMotorDirPin1 ,LOW);
   digitalWrite(slideMotorDirPin2 ,HIGH); 

}
void retractSlide() {
   slideRetracting = true;
   digitalWrite(slideMotorDirPin1 ,HIGH); 
   digitalWrite(slideMotorDirPin2 ,LOW);
}

void stopSlide() {
   // If the slide is moving wait for the next interrupt sync so we stop right when the hall-sensor detects a magnet.  
   // That way we don't accumulate run-out error if the user bangs the stick back and forth between sensor ticks.
   if(slideExtending || slideRetracting) {
      interruptSync = false;
      if(DEBUG) {
         Serial.print(F(" Waiting for interrupt"));
      }
      while(!interruptSync) {
      }
      Serial.println("");
   }
   digitalWrite(slideMotorDirPin1 ,LOW); 
   digitalWrite(slideMotorDirPin2 ,LOW);
   if(DEBUG) {
      Serial.print(F("SlideMotorCount: "));
      Serial.println(slideMotorCounter);
   }
   slideExtending = false;
   slideRetracting = false;
}

void raiseSlide() {
   digitalWrite(linearActuatorDirPin1 ,HIGH); 
   digitalWrite(linearActuatorDirPin2 ,LOW);
}

void lowerSlide() {
   digitalWrite(linearActuatorDirPin1 ,LOW);
   digitalWrite(linearActuatorDirPin2 ,HIGH); 
}


// #####################################################
// Send message back to the transmitter (client)
// #####################################################
// !!!! This does not work reliably.  The radio gets sendToWait errors when trying to send 
//      packets to the joystick-transmitter.  Don't use until I clean up the robot radio.
//      I've already tried all kinds of combinations of lower datarate and higher and lower xmit power.
void sendMessageToClient(char * messageBuffer) {
   if(!manager.sendtoWait((uint8_t *)&messageBuffer, strlen(messageBuffer), CLIENT_ADDRESS)) {
      Serial.println(F("sendtoWait(messageToXmitter) failed"));
   }
}
// #####################################################
// Check the state of a switch after debouncing it
// #####################################################
boolean checkSwitch(int pin, boolean doDebounceCheck) {
    boolean state;
    boolean prevState;
    int debounceDelay = 20;
    prevState = digitalRead(pin);
    if(doDebounceCheck) {
       for(int counter=0; counter < debounceDelay; counter++) {
           delay(1);
           state = digitalRead(pin);
           if(state != prevState) {
               counter=0;
               prevState=state;
           }
       }
       // At this point the switch state is stable
       if(state == HIGH) {
           return true;
       } else {
           return false;
       }
    } else {
      return(prevState);
    }
}

// Check switch that is connected to A6 or A7.  A6/A7 can only use analogRead so we have to determine if a 1 or 0 is being read.
// doDebounceCheck switch set false when we just want to do a raw read with no debounce (used when we only care about the triggering edge)
boolean checkSwitchA6A7(int pin, boolean doDebounceCheck) {
    int ain;
    boolean state;
    boolean prevState;
    int debounceDelay = 20;
    ain=analogRead(pin);
    if(ain > 512) {
       prevState = 1;
    } else {
       prevState = 0;
    }
    if(doDebounceCheck) {
        for(int counter=0; counter < debounceDelay; counter++) {
            delay(1);
            ain=analogRead(pin);
            if(ain > 512) {
                state = 1;
             } else {
                state = 0;
             }
            if(state != prevState) {
                counter=0;
                prevState=state;
            }
        }
        // At this point the switch state is stable
        if(state == HIGH) {
            return true;
        } else {
            return false;
        }
    } else {
      return(prevState);
    }
}



//##############################################################################
//##############################################################################
// Main code
//##############################################################################
//##############################################################################
void setup() {
   Serial.begin(115200);  // For debugging monitor
   delay(1000);  

   ; // Wait for serial port to connect. 
   if (!manager.init()) {
      Serial.println(F("init failed"));
   }

   // Initialize pins
   pinMode(speedPinL, OUTPUT);  
   pinMode(speedPinR, OUTPUT);
   pinMode(rightMotorDirPin1, OUTPUT);
   pinMode(rightMotorDirPin2, OUTPUT);
   pinMode(leftMotorDirPin1, OUTPUT);
   pinMode(leftMotorDirPin2, OUTPUT);
   pinMode(slideMotorDirPin1, OUTPUT);
   pinMode(slideMotorDirPin2, OUTPUT);
   pinMode(rotatorDirPin1, OUTPUT);
   pinMode(linearActuatorDirPin1, OUTPUT);
   pinMode(linearActuatorDirPin2, OUTPUT);
   pinMode(headlightControl, OUTPUT);
   pinMode(slideMotorHallPin, INPUT_PULLUP);

   // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
   if (!driver.setChannel(radioChannel)) {
      Serial.println(F("setChannel failed"));
   }
   // This turn down the datarate to 1Mbps, and lowers the power to -12dB to to minimize radio noise issues
   if (!driver.setRF(RH_NRF24::DataRate1Mbps, RH_NRF24::TransmitPowerm12dBm)){
      if(DEBUG) {
         Serial.println(F("setRF failed"));
         driver.printRegisters();
      }
   } else {
      if(DEBUG) {
         Serial.println(F("Radio Initialized"));
      }
   }

   allStop();//stop move
   digitalWrite(rotatorDirPin1, LOW);  // Turn off the rotisserie

   // Register the slide motor hall sensor interrupt
   attachInterrupt(digitalPinToInterrupt(slideMotorHallPin), updateSlidePulleyCount, FALLING);

   // Home the slide (resetting the motor counter to zero)
   homeTheSlide();
}

// #####################################################
// Main loop
// #####################################################
void loop() {

   // Safety check.  Stop the slide if we ever hit the limit switch
   if(checkSwitchA6A7(slideLimitSwitchPin,false) == SWITCH_PRESSED) {

      // Immediately stop the motor
      digitalWrite(slideMotorDirPin1 ,LOW); 
      digitalWrite(slideMotorDirPin2 ,LOW);
      slideMotorCounter = 0;
      slideExtending = false;
      slideRetracting = false;
      if(DEBUG) {
         Serial.println(F("Unexpectantly found limit switch, Re-homing slide"));
      }
      homeTheSlide();
   }
   // If the slides are moving check that they have not exceeded the travel limits
   if((slideExtending && slideMotorCounter >= MAX_SLIDE_MOTOR_COUNT-1) || (slideRetracting && slideMotorCounter <= MIN_SLIDE_MOTOR_COUNT+1)) {
      if(DEBUG) {
         Serial.print(F("Slide Motor Limit Reached: "));
         Serial.println(slideMotorCounter);
      }
      stopSlide();
   }

   // Each time through the loop see if we have any joystick data packets from the transmitter
   if (manager.available()) {
      newPacketAvailable = true;
      uint8_t len = sizeof(DataStructure);

      // The magic here is casting a (uint8_t *) pointer and passing the packet structure address to the receive method
      // Without the cast, we would get a mismatch of data-type (recv expects byte pointer, not a structure)...
      if (manager.recvfromAck((uint8_t *)&packet, &len)) {

         // Toggle the headlights on/off
         if(packet.joyAddress == 0 && packet.joySwitch == 0) {  
            headlightsOn = !headlightsOn;
            if(headlightsOn) {
               digitalWrite(headlightControl, HIGH);
            } else {
               digitalWrite(headlightControl, LOW);
            }
         }
         // Toggle marshmallow rotator motor on/off when left joystick switch pushed
         if(packet.joyAddress == 1 && packet.joySwitch == 0) {
            rotatorMotorOn = !rotatorMotorOn;
            if(rotatorMotorOn) {
               digitalWrite(rotatorDirPin1, HIGH);
               Serial.println("Rotator motor on");
            } else {
               digitalWrite(rotatorDirPin1, LOW);
               Serial.println("Rotator motor off");
            }
         }
      } 
   } else {
      newPacketAvailable = false;
   }

   if(newPacketAvailable) {

      //###########################################################################
      // Read the joysticks and control the motors
      //###########################################################################

      if(DEBUG) {
         Serial.print("joyAddress: "); Serial.println(packet.joyAddress,DEC);
         Serial.print("joyX: "); Serial.println(packet.joyX,DEC);
         Serial.print("joyY: "); Serial.println(packet.joyY,DEC);
         Serial.print("joyXCenter: "); Serial.println(packet.joyXCenter,DEC);
         Serial.print("joyYCenter: "); Serial.println(packet.joyYCenter,DEC);
         Serial.print("joySwitch: "); Serial.println(packet.joySwitch,DEC);
         Serial.println();
      }


      int goSpeed, leftWheelSpeed, rightWheelSpeed;;
      const uint8_t margin = JOY_MARGIN;

      // Address0 is the robot wheels joystick, address1 is the robot arm joystick
      if(packet.joyAddress == 0) {   
         if(packet.joyX > (packet.joyXCenter + margin) && 
               abs(packet.joyY - packet.joyYCenter) < margin) {
            spin_cw();

            goSpeed = map(packet.joyX,packet.joyXCenter,1023,0,FULL_SPEED);
            if(goSpeed > FULL_SPEED) { goSpeed = FULL_SPEED; }
            set_MotorSpeed(goSpeed,goSpeed);
            if(DEBUG) {
               Serial.print(F("Spin CW ")); Serial.println(goSpeed); 
               //  sendMessageToClient("spinCW");
            }

         } else if(packet.joyX < (packet.joyXCenter - margin) &&
               abs(packet.joyY - packet.joyYCenter) < margin) {
            spin_ccw();

            goSpeed = map(packet.joyX,0,packet.joyXCenter,FULL_SPEED,0);
            if(goSpeed > FULL_SPEED) { goSpeed = FULL_SPEED; }
            set_MotorSpeed(goSpeed,goSpeed);
            if(DEBUG) {
               Serial.print(F("spin_ccw ")); Serial.println(goSpeed); 
               //  sendMessageToClient("spinCCW");
            }

            // Going forward (maybe veering left or right)
         } else if(packet.joyY > packet.joyYCenter + margin) {
            go_Forward();

            // Turning forward-right (by slowing down the right wheel)
            if(packet.joyX - packet.joyXCenter > 0 + margin) {
               leftWheelSpeed = map(packet.joyY,packet.joyYCenter,1023,0,FULL_SPEED);
               if(leftWheelSpeed > FULL_SPEED) { leftWheelSpeed = FULL_SPEED; }
               rightWheelSpeed = map(packet.joyX,packet.joyXCenter,1023,leftWheelSpeed,0); 
               set_MotorSpeed(leftWheelSpeed,rightWheelSpeed);
               if(DEBUG) {
                  Serial.print(F("goForwardRight ")); Serial.print(leftWheelSpeed); Serial.print(" "); Serial.println(rightWheelSpeed);
                  //  sendMessageToClient("goForwardRight");
               }

               // Turning forward-left (by slowing down the left wheel)
            } else if(packet.joyX - packet.joyXCenter < 0 - margin) {
               rightWheelSpeed = map(packet.joyY,packet.joyYCenter,1023,0,FULL_SPEED);
               if(rightWheelSpeed > FULL_SPEED) { rightWheelSpeed = FULL_SPEED; }
               leftWheelSpeed = map(packet.joyX,packet.joyXCenter,0,rightWheelSpeed,0); 
               set_MotorSpeed(leftWheelSpeed,rightWheelSpeed);
               if(DEBUG) {
                  Serial.print(F("goForwardLeft ")); Serial.print(leftWheelSpeed); Serial.print(" "); Serial.println(rightWheelSpeed);
                  //  sendMessageToClient("goForwardLeft");
               }

               // Going forward-straight
            } else {
               goSpeed = map(packet.joyY,packet.joyYCenter,1023,0,512);
               if(goSpeed > FULL_SPEED) { goSpeed = FULL_SPEED; }
               set_MotorSpeed(goSpeed,goSpeed);
               if(DEBUG) {
                  Serial.print(F("goForward ")); Serial.print(goSpeed); Serial.print(" "); Serial.println(goSpeed);
                  //  sendMessageToClient("goForward");
               }
            }

            // Going backward (maybe veering left or right)
         } else if(packet.joyY < packet.joyYCenter - margin) {
            go_Back();

            // Turning backward-right (by slowing down the right wheel)
            if(packet.joyX - packet.joyXCenter > 0 + margin) {
               leftWheelSpeed = map(packet.joyY,0,packet.joyYCenter,FULL_SPEED,0);
               if(leftWheelSpeed > FULL_SPEED) { leftWheelSpeed = FULL_SPEED; }
               rightWheelSpeed = map(packet.joyX,packet.joyXCenter,1023,leftWheelSpeed,0); 
               set_MotorSpeed(leftWheelSpeed,rightWheelSpeed);
               if(DEBUG) {
                  Serial.print(F("goBackwardRight ")); Serial.print(leftWheelSpeed); Serial.print(" "); Serial.println(rightWheelSpeed);
                  //  sendMessageToClient("goBackwardRight");
               }

               // Turning backward-left (by slowing down the left wheel)
            } else if(packet.joyX - packet.joyXCenter < 0 - margin) {
               rightWheelSpeed = map(packet.joyY,0,packet.joyYCenter,FULL_SPEED,0);
               if(rightWheelSpeed > FULL_SPEED) { rightWheelSpeed = FULL_SPEED; }
               leftWheelSpeed = map(packet.joyX,packet.joyXCenter,0,rightWheelSpeed,0); 
               set_MotorSpeed(leftWheelSpeed,rightWheelSpeed);
               if(DEBUG) {
                  Serial.print(F("goBackwardLeft ")); Serial.print(leftWheelSpeed); Serial.print(" "); Serial.println(rightWheelSpeed);
                  //  sendMessageToClient("goBackwardLeft");
               }

               // Going backward-straight
            } else {
               goSpeed = map(packet.joyY,0,packet.joyYCenter,FULL_SPEED,0);
               if(goSpeed > FULL_SPEED) { goSpeed = FULL_SPEED; }
               set_MotorSpeed(goSpeed,goSpeed);
               if(DEBUG) {
                  Serial.print(F("goBackward ")); Serial.print(goSpeed); Serial.print(" "); Serial.println(goSpeed);
                  // sendMessageToClient("goBackward");
               }
            }

            // Joysticks centered.  Stop 
         } else {
            allStop();
            //Serial.println(F("Stop")); 
            //sendMessageToClient("Stop");
         }
      }

      // If we're getting robot-arm commands
      if(packet.joyAddress == 1) {

         // Stick forward,  extend the slide
         if(packet.joyY > packet.joyYCenter + margin) {
            if(DEBUG) {
               Serial.println(F("Extend slide"));
               //  sendMessageToClient("Extend slide");
            }
            if(slideMotorCounter < MAX_SLIDE_MOTOR_COUNT) {
               extendSlide();
            } else {
               if(DEBUG) {
                  Serial.println(F("Can't extend slide. Travel limit reached."));
               }
            }

            // Stick back,  retract the slide
         } else if(packet.joyY < packet.joyYCenter - margin) {
            if(DEBUG) {
               Serial.println(F("Retract slide"));
               //  sendMessageToClient("Retract slide");
            }
            if(slideMotorCounter > MIN_SLIDE_MOTOR_COUNT) {
               retractSlide();
            } else {
               if(DEBUG) {
                  Serial.println(F("Can't retract slide. Travel limit reached."));
               }
            }

            // Stick left/right -  Raise/Lower the arm
         } else if(packet.joyX > packet.joyXCenter + margin) {
            lowerSlide();
            if(DEBUG) {
               Serial.print(F("Lower Slide ")); Serial.println(goSpeed);
               //  sendMessageToClient("Lower Slide");
            }
         } else if(packet.joyX < packet.joyXCenter - margin) {
            if(DEBUG) {
               Serial.print(F("Raise Slide ")); Serial.println(goSpeed);
               //  sendMessageToClient("Raise Slide");
            }
            raiseSlide();
         } else {
            if(DEBUG) {
               Serial.println(F("Stop "));
               // sendMessageToClient("Stop");
            }
            allStop();
         }
      }
   }
}

