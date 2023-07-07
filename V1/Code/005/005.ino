#include <IBusBM.h>
IBusBM IBus; // IBus object

// ramp lib
#include <Ramp.h>

#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;

Servo servo4;
Servo servo5;
Servo servo6;

Servo servo7;
Servo servo8;
Servo servo9;

int ch1;
int ch2;
int ch3;
int ch4;
int ch5;
int ch6;
int ch7;
int ch8;
int ch9;
int ch10;

float RFB;
float RFBa;
float RFBFiltered;
float RLR;
float RLRa;
float RLRFiltered;
float RT;
float RTa;
float RTFiltered;

float LFB;
float LFBa;
float LFBFiltered;
float LLR;
float LLRa;
float LLRFiltered;
float LT;
float LTa;
float LTFiltered;

float pos1;
float pos2;
float pos3;
float pos4;
float pos5;
float pos6;
float pos7;
float pos8;
float pos9;


float pos1Offset;
float pos2Offset;
float pos3Offset;
float pos4Offset;
float pos5Offset;
float pos6Offset;
float pos7Offset;
float pos8Offset;
float pos9Offset;


float pos1Filtered;
float pos2Filtered;
float pos3Filtered;
float pos4Filtered;
float pos5Filtered;
float pos6Filtered;
float pos7Filtered;
float pos8Filtered;
float pos9Filtered;

float x;
float z;
float za;
float xa;
float y;
float ya;
float xb1;
float xb2;
float xb3;
float yb1;
float yb2;
float yb3;
float zb1;
float zb2;
float zb3;

float xOffset;
float yOffset;

float leg1z;
float leg2z;
float leg3z;
float leg4z;
float leg1x;
float leg2x;
float leg3x;
float leg4x;
int leg1y;
int leg2y;
int leg3y;
int leg4y;

float stride;
int stride2;

unsigned long currentMillis;
long previousMillis = 0;        // set up timers
long interval = 10;             // time constant for timer

long previousInterpMillis = 0;  // set up timers for interpolation
int interpFlag = 0;

int stepFlag = 0;
long previousStepMillis = 0;    // timers for walking states 
int stepStartFlag = 0;
int duration;
int timer1;

int duration2;
int timer2;


int stepFlag2 = 0;
long previousStepMillis2 = 0;    // timers for walking states 

float walkXPos1;
float walkXPos2;
float walkXPos3; 
float walkXPos4; 
float walkXPos5;
float walkXPos6; 
float walkXPos7; 
float walkXPos8;

float walkYPos1;
float walkYPos2;
float walkYPos3; 
float walkYPos4; 

float walkZPos1;
float walkZPos2;
float walkZPos3;
float walkZPos4;
float walkZPos5;
float walkZPos6;
float walkZPos7;
float walkZPos8;


class Interpolation {
  public:
    rampInt myRamp;
    int interpolationFlag = 0;
    int savedValue;

    int go(int input, int duration) {

      if (input != savedValue) {   // check for new data
        interpolationFlag = 0;
      }
      savedValue = input;          // bookmark the old value

      if (interpolationFlag == 0) {                                   // only do it once until the flag is reset
        myRamp.go(input, duration, LINEAR, ONCEFORWARD);              // start interpolation (value to go to, duration)
        interpolationFlag = 1;
      }

      int output = myRamp.update();
      return output;
    }
};    // end of class

Interpolation interpFRX;        // interpolation objects
Interpolation interpFRY;
Interpolation interpFRZ;

Interpolation interpFLX;        // interpolation objects
Interpolation interpFLY;
Interpolation interpFLZ;

Interpolation interpBRX;        // interpolation objects
Interpolation interpBRY;
Interpolation interpBRZ;

Interpolation interpBLX;        // interpolation objects
Interpolation interpBLY;
Interpolation interpBLZ;

void setup() {

    Serial.begin(115200);
    
    IBus.begin(Serial3, IBUSBM_NOTIMER);    // change to Serial1 or Serial2 port when required
      
    servo1.attach(22);
    servo2.attach(24); 
    servo3.attach(26); 
    servo4.attach(28);
    servo5.attach(30); 
    servo6.attach(32);
    
    servo7.attach(34);
    servo8.attach(36); 
    servo9.attach(38);

    // offsets for procesise home positions
        
    pos1Offset = 30;
    pos2Offset = -55;
    pos3Offset = -40;
    pos4Offset = 60;
    pos5Offset = -10;
    pos6Offset = 20;
    pos7Offset = 50;
    pos8Offset = 50;
    pos9Offset = 100;
}

void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {  // start timed event    
      previousMillis = currentMillis;

      IBus.loop();

      ch1 = IBus.readChannel(0); // get latest value for servo channel 0
      ch2 = IBus.readChannel(1); // get latest value for servo channel 1
      ch3 = IBus.readChannel(2); // get latest value for servo channel 3
      ch4 = IBus.readChannel(3); // get latest value for servo channel 4
      ch5 = IBus.readChannel(4); // get latest value for servo channel 5
      ch6 = IBus.readChannel(5); // get latest value for servo channel 6
      ch7 = IBus.readChannel(6); // get latest value for servo channel 7
      ch8 = IBus.readChannel(7); // get latest value for servo channel 8
      ch9 = IBus.readChannel(8); // get latest value for servo channel 9
      ch10 = IBus.readChannel(9); // get latest value for servo channel 10

      LFB = ch1;
      RFB = ch4;
      RLR = ch2;
      RT = ch6;
      LLR = ch3;
      LT = ch5;
      
      // threshold sticks

      RFBa = thresholdStick(RFB);
      RLRa = thresholdStick(RLR);
      RTa = thresholdStick(RT);
      LFBa = thresholdStick(LFB);
      LLRa = thresholdStick(LLR);
      LTa = thresholdStick(LT);

      // filter sticks

      RFBFiltered = filter(RFBa, RFBFiltered,20);
      RLRFiltered = filter(RLRa, RLRFiltered,20);
      RTFiltered = filter(RTa, RTFiltered,20);
      LFBFiltered = filter(LFBa, LFBFiltered,20);
      LLRFiltered = filter(LLRa, LLRFiltered,20);
      LTFiltered = filter(LTa, LTFiltered,20);

      if (ch7 < 1300) {               // kinematics test mode
          za =  map(LTFiltered,-255,255,130,240);
          za = za - 16;
          xa =  map(RFBFiltered,-255,255,100,-100);
          ya =  map(RLRFiltered,-255,255,100,-100);
          xa = constrain(xa,-100,100);
          ya = constrain(ya,-100,100);

          stepFlag = 0;
    
          z = (float) za/1000;
          x = (float) xa/1000;
          y = (float) ya/1000;

          xOffset = 0.05;
          yOffset = 0.04;

          kinematics(1, x-xOffset, y ,z);
          kinematics(2, x+xOffset, y+0.04 ,z);
          kinematics(3, x+xOffset, y+-0.04 ,z);
      }

      else if (ch7 > 1400 && ch7 < 1700) {                          // walking mode
      
          if (stepFlag == 0 && currentMillis - previousStepMillis > 900) {         
              previousStepMillis = currentMillis;
              stepFlag = 1;
              xb1 = -0.035;
              xb2 = -0.035;
              xb3 = -0.035;
              yb1 = 0;
              yb2 = 0;
              yb3 = 0;
              zb1 = 0.16;
              zb2 = 0.16;
              zb3 = 0.16;
          }  
          else if (stepFlag == 1 && currentMillis - previousStepMillis > 200) {     // step 1 starts
              previousStepMillis = currentMillis;          
              stepFlag = 2;
              xb1 = -0.015;
              xb2 = -0.045;
              xb3 = -0.045;
              yb1 = 0;
              yb2 = 0;
              yb3 = 0;
              zb1 = 0.14;
              zb2 = 0.16;
              zb3 = 0.16;
          }
          else if (stepFlag == 2 && currentMillis - previousStepMillis > 200) {     // step 1 fininishes
              previousStepMillis = currentMillis;          
              stepFlag = 3;
              xb1 = -0.015;
              xb2 = -0.045;
              xb3 = -0.045;
              yb1 = 0;
              yb2 = 0;
              yb3 = 0;
              zb1 = 0.16;
              zb2 = 0.16;
              zb3 = 0.16;
          }
          else if (stepFlag == 3 && currentMillis - previousStepMillis > 900) {
              previousStepMillis = currentMillis;          
              stepFlag = 4;
              xb1 = 0.005;
              xb2 = -0.025;
              xb3 = -0.025;
              yb1 = 0.03;
              yb2 = 0.03;
              yb3 = 0.03;
              zb1 = 0.16;
              zb2 = 0.16;
              zb3 = 0.16;
          }
          else if (stepFlag == 4 && currentMillis - previousStepMillis > 200) {       // step 2 starts
              previousStepMillis = currentMillis;          
              stepFlag = 5;
              xb1 = 0.005;
              xb2 = 0.015;
              xb3 = -0.025;
              yb1 = 0.03;
              yb2 = 0.03;
              yb3 = 0.03;
              zb1 = 0.16;
              zb2 = 0.14;
              zb3 = 0.16;              
          }
          else if (stepFlag == 5 && currentMillis - previousStepMillis > 200) {     // step 2 finishes
              previousStepMillis = currentMillis;          
              stepFlag = 6;
              xb1 = 0.005;
              xb2 = 0.015;
              xb3 = -0.025;
              yb1 = 0.03;
              yb2 = 0.03;
              yb3 = 0.03;
              zb1 = 0.16;
              zb2 = 0.16;
              zb3 = 0.16;              
          }
          else if (stepFlag == 6 && currentMillis - previousStepMillis > 900) {
              previousStepMillis = currentMillis;          
              stepFlag = 7;
              xb1 = 0.015;
              xb2 = 0.025;
              xb3 = -0.025;
              yb1 = -0.01;
              yb2 = -0.01;
              yb3 = -0.01;
              zb1 = 0.16;
              zb2 = 0.16;
              zb3 = 0.16;              
          }
          else if (stepFlag == 7 && currentMillis - previousStepMillis > 200) {     // step 3 starts
              previousStepMillis = currentMillis;          
              stepFlag = 8;
              xb1 = 0.015;
              xb2 = 0.025;
              xb3 = 0.055;
              yb1 = -0.01;
              yb2 = -0.01;
              yb3 = -0.01;
              zb1 = 0.16;
              zb2 = 0.16;
              zb3 = 0.14;              
          }
          else if (stepFlag == 8 && currentMillis - previousStepMillis > 200) {     // step 3 finishes
              previousStepMillis = currentMillis;          
              stepFlag = 0;
              xb1 = 0.015;
              xb2 = 0.025;
              xb3 = 0.055;
              yb1 = -0.01;
              yb2 = -0.01;
              yb3 = -0.01;
              zb1 = 0.16;
              zb2 = 0.16;
              zb3 = 0.16;              
          }

          

          xOffset = 0.05;
          
          kinematics(1, xb1-xOffset, yb1 ,zb1);
          kinematics(2, xb2+xOffset, yb2+0.04 ,zb2);
          kinematics(3, xb3+xOffset, yb3+-0.04 ,zb3);
      } 


  }   // end of timed loop

}




