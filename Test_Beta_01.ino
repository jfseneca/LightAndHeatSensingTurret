/*** Header Block ***
   Code file name:
      Test_Beta_##
   Code description:
      -Unit test program to test and debug final code
      -(1)Controls tilt and pan with the axis of the joystick 
      ---From Test_ServoJoystick_Control_03
      -(2)Tracks single lightsources
      ---From Test_PixyCam2_Tracking_04f
      -(3)Compares multiple light sources by temp and aims at the highest temp source
      ---From Test_PixyCam2_SweepTrack_04d
     -V00: 12/13/21 Combining code into one file
   Hardware components required with this code:
      -Servomotors
      -A buck converter (to supply a steady 5v and amperage)
      -Wires
      -Joystick (is Active-LOW [+5v unpressed, 0v depressed])
      -Pssive Buzzer (Analog Unit)
      -100 OHM Resistor (Btwn Buzzer Pin and GND)
      -2 DC motors to fire darts
      -Arduino (My model is the Nano Every)
      -Servomotors
      -A buck converter (to supply a steady 5v and amperage to motors)
      -Wires
      -PixyCam2
   IDE version used to test code:
      Arduino IDE v1.8.16
   Programmer(s) name: Herbert Furman
   Date when code is created/modified: Cr:12/13/21 
   Code version: 00

  Based on:
  
  Test_ServoJoystick_Control_03,From Test_PixyCam2_Tracking_04g,From Test_PixyCam2_SweepTrack_04d
  by Herbert Furman ***(See individual code files for credit to other's work)
  ***/
/****************************************/

// Pre-processor Directives
#include <Servo.h> //controls servos
#include <Pixy2SPI_SS.h> //Take in data from PixyCam2 via Arduino SPI interface
#include <Arduino_Helpers.h> //AH allows use of C structs
#include <AH/STL/algorithm>
#include <AH/STL/iterator>
#include <OneButton.h>  //Extra functionality for buttons
#include "pitches.h"    //To create tones with active buzzer

// Class/Object Declarations
//-----Configure Servos
Servo recoil_servo;
Servo pan_servo;
Servo tilt_servo;
Pixy2SPI_SS pixy; //Create PixyCam2 object using the SPI/Arduino interface.

//-----Struct for storing infor on each light
struct light_type{
    int angle_x; //The angle of the servo used to aim at the object, using the map function
    int angle_y;
    int temp; //the temperature of the object
  };

light_type light[2]; //Create a struct with space for 3 lights

//-----Configure Button
#define BUTTON_PIN 4
OneButton SW1 = OneButton(
  BUTTON_PIN,  // Input pin for the button
  true,        // Button is active LOW
  true         // Enable internal pull-up resistor
);

//----------The actions the button can take.
typedef enum {
  ACTION_OFF,  // set LED "OFF".
  ACTION_ON,   // set LED "ON"
  ACTION_SLOW, // blink LED "SLOW"
  ACTION_FAST  // blink LED "FAST"
} 
MyActions;

//-----Melodie(s) for tones
int buzzPin=2;
int dt1=500;
int dt2=250;

int melody0[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

int melody1[] = {
  NOTE_B3, NOTE_B3
};

int melody2[] = {
  NOTE_G3, NOTE_G3
};

//-----Note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDuration1[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

int noteDuration2[] = {
  4, 4
};

//Global variables
/***
    Limits  Angles  MicroS  Note
    Recoil  0               is closest to dc motors    
            70      1200    "full stroke"
            120     ????    "at rest" (primed)
    Pan     90      1500    "rest" (centered)
            30      1100    To the "right" (towards mainboard)
            150     1900    To the "left"
    Tilt    5      ????    "Down" (towards ground)
            43       900    "at rest" (parallel to earth)
            75      ????    "Up" (towards sky)
            ??    
***/
//-----Servo global variables
int recoil_rest = 120;   
int recoil_pushed = 70;  
int recoil_pos = recoil_rest;
int pan_rest = 90;
int pan_limit_1 = 30;
int pan_limit_2 = 150;
int pan_pos = pan_rest;
int tilt_rest = 35;
int tilt_limit_1 = 10;
int tilt_limit_2 = 120;
int tilt_pos = tilt_rest;
int speed1 = 5;   // increase speed variable to decrease speed actual
int steps = 1;    //increase step variiable to increase speed actual

//-----Variable related to DC motor
int motor_pin =  10;
boolean motors_ON = false;

//-----Joystick global variiables
int p1 = pan_rest;
int p2 = tilt_rest; //position variable

const int y = 1; // Connect to Analog Pin
const int x = 2; // Connect to Analog Pin

//----------Joystick button 
MyActions nextAction = ACTION_OFF; // no action when starting

//-----Configure PID controls
float pan_k1, tilt_k1;     //Proportionaliity
float pan_k2, tilt_k2;     //Derivative
float pan_k3, tilt_k3;  //Integral

int milliOld, milliNew, dt; //Old time, new time, difference btwn old and new

float panSetpoint, tiltSetpoint; //Where we want the servo to be
float panInput, tiltInput; //Where the servo is
float panError=0, tiltError=0; //The difference between where the servo is and where it should be
float panErrorOld, tiltErrorOld; //The old Error
float panErrorChange, tiltErrorChange; //How the error is changing over time
float panErrorSlope=0, tiltErrorSlope=0; //The slope of the error (for Derivative)
float panErrorArea=0, tiltErrorArea=0; //The area under the curve of the error (for Integral)
float panServoVal=0, tiltServoVal=0; //The new value to move to after PID

unsigned long lastUpdate,lastUpdate1,lastUpdate2,lastUpdate3; // last update of position

//-----Flame Sensor
const int a0Pin = A0;

//-----misc
int blocknum=0; //The block number to focus on. Each block is a light source.
int SWITCH=0; //Change between states.
int interval1; //The interval at which to update the x and y direction.

void setup() {
  // Configure hardware peripherals
  //-----attaches servo to pins
  recoil_servo.attach(7);
  recoil_servo.write(recoil_rest);
  pan_servo.attach(6);
  pan_servo.write(pan_rest);
  tilt_servo.attach(9);
  tilt_servo.write(tilt_rest);

  //-----Zero Out
  zero_out();

  //-----Define Motor pin mode
  pinMode(motor_pin, OUTPUT);
  digitalWrite(motor_pin, LOW);

  //-----Configure Buzzer pin
  pinMode(buzzPin,OUTPUT);
  
  //-----Configures Button
  SW1.attachClick(myClickFunction);
  SW1.attachDoubleClick(myDoubleClickFunction);// link the doubleclick function to be called on a doubleclick event
  SW1.attachLongPressStop(myLongPressStopFunction);
  SW1.attachMultiClick(myMulticlickFunction);
  //SW1.setDebounceTicks(100); //Period of time in which to ignore additional level changes.
  SW1.setClickTicks(250); //Distinguish clicks from double.c Default:500msec
  
  //Configure Data Communication
  Serial.begin(19200);  //Start Serial Monitor
  
  // Code that runs only once
  Serial.print("Starting...\n");
  pixy.init();
}

void loop() {
  SW1.tick(); //In order for OneButton to work correctly, 
  //you must call tick() on each button instance within 
  //your main loop(). If you're not getting any button 
  //events, this is probably why.
  uint16_t blocks;  //block holds tracked data for the pixy
  char buf[32];     //pixy data buffer

  //Joystick control.
  if(SWITCH == 0){
    int t1=0,t2=0;  //rate of increment/decrement of angle
    int a=analogRead(x); // reads analog x -y readings of joystick
    int b=analogRead(y);
  
    //when joystick is moved away from the center
    if(a<=625 or a>=660){
      t1=map(a,0,1023,5,-5);
      p1=change_pan(p1,t1);  //change the servo's current position
    }
    if(b<=625 or b>=660){
      t2=map(b,0,1023,-5,5);
      p2=change_tilt(p2,t2);  //change the servo's current position
    }
  
    pan_servo.write(p1); // rotate the servo's if the joystick is moved
    tilt_servo.write(p2);
  
    //print the analog readings of joystick and servo degree(For Verification)
    //-----\t is tab
    Serial.print("pan_servo (x):"); Serial.print(a);
    Serial.print("\t"); Serial.print("tilt_servo (y):"); Serial.print(b);
    Serial.print("\t"); Serial.print("t1:"); Serial.print(t1);
    Serial.print("\t"); Serial.print("t2:"); Serial.print(t2);
    Serial.print("\t"); Serial.print("pos1: "); Serial.print(p1);
    Serial.print("\t"); Serial.print("pos2: "); Serial.println(p2);
  }
  
  //Singular light source tracking.
  else if(SWITCH == 1){
    blocks = pixy.ccc.getBlocks(); 
    Serial.println("In case 1: Single source tracking.");
    milliOld=milliNew;
    Serial.print("milliOld :");Serial.println(milliOld);
    milliNew=millis();
    Serial.print("milliNew = :");Serial.println(milliNew);
    dt=milliNew-milliOld;
    Serial.print("dt = :");Serial.println(dt);
    interval1=100;
    pan_k1=0.25; tilt_k1=0.1;     //Proportionaliity
    pan_k2=0.0; tilt_k2=0.0;     //Derivative
    pan_k3=.00001; tilt_k3=.00001;  //Integral
    if(pixy.ccc.numBlocks){
      Serial.println("****************************************");
      x_update(); //move pan servo to the center of the block.
      y_update(); //move tilt servo to the center of the block.
    }
  }

  //Multiple light source temp comparator.
  else if(SWITCH == 2){
    blocks = pixy.ccc.getBlocks(); 
    Serial.println("In case 2: Multiple source comparator.");
    milliOld=milliNew;
    Serial.print("milliOld :");Serial.println(milliOld);
    milliNew=millis();
    Serial.print("milliNew = :");Serial.println(milliNew);
    dt=milliNew-milliOld;
    Serial.print("dt = :");Serial.println(dt);
    interval1=100; //in miliseconds
    pan_k1=0.1; tilt_k1=0.1;     //Proportionaliity
    pan_k2=0.0; tilt_k2=0.0;     //Derivative
    pan_k3=.00001; tilt_k3=.00001;  //Integral
    if(pixy.ccc.numBlocks){
      Serial.println("********************************************************************************");
      Serial.print("num of blocks");Serial.println(pixy.ccc.numBlocks);
      Serial.print("Blocknum "); Serial.println(blocknum);
      Serial.print("Error ");Serial.print(abs(panError));Serial.print(", ");Serial.println(abs(tiltError));
      x_update(); //move pan servo to the center of the block.
      y_update(); //move tilt servo to the center of the block.
      num_update(); //record location and temp of that block.
      fire_update(); //fire at highest temp block.
    }
  }
}

//----------User-Defined Functions
//-----Handler Functions for clicking the button and switches states
//Handler function for any click:
static void handleClick() {
  Serial.println("Clicked!");
}

//Click once to activate.
void myMulticlickFunction() {
  Serial.println("myClickFunction (single click) activated.");
  if(SWITCH == 0){
    cycle_recoil();
  }
}

//This function will be called when the button was pressed 2 times in a short timeframe.
//Switches states.
void myDoubleClickFunction() {
    Serial.println("myDoubleClickFunction (double click) activated.");
  if (motors_ON == false){
    digitalWrite(motor_pin, HIGH);
    motors_ON = true;
  }
  else{
    digitalWrite(motor_pin, LOW);
    motors_ON = false;
  }
} 

//Three button presses to activate.
//Turns on firing motors.
void myClickFunction(){
      blocknum=0;
      zero_out();
      panError=0;
      tiltError=0;
      p1=0;
      p2=0;
      //milliNew=0;
      lastUpdate=millis();
      lastUpdate1=millis();
      lastUpdate2=millis();
      lastUpdate3=millis();
      Serial.println("In the multi-click function");
      if(SWITCH == 0){
        playMelody(melody1,noteDuration2);
        blocknum = 0;
        SWITCH = 1;
      }
    else if(SWITCH==1){
      playMelody(melody2,noteDuration2);
      SWITCH = 2;
    }
    else if(SWITCH==2){
      playMelody(melody2,noteDuration2);
      SWITCH = 0;
    }
}

//Click and hold for some time to activate.
//"Zeros" (centers) servos.
void myLongPressStopFunction(){
  zero_out();
}

//-----Related to state 0
//Fires a dart, cycling the recoil sequence
int cycle_recoil(){ 
    recoil_servo.write(recoil_pushed);
    delay(750);
    recoil_servo.write(recoil_rest);
    delay(15);
}

//Centers motors.
void zero_out(){
    //-----Zero Out
  recoil_servo.write(recoil_rest);
  delay(500);
  pan_servo.write(pan_rest);
  delay(500);
  tilt_servo.write(tilt_rest);
  delay(500);
}

//Pan servo. Used w/ joystick.
int change_pan(int pos,int t){
    pos=pos+t;                   // Increment/decrement the Servo angle
    if(pos<pan_limit_1)                  // minimum angle of servo is 0 degree
      pos=pan_limit_1;
    if (pos>pan_limit_2)                //  maximum angle of servo is 180 degree
      pos=pan_limit_2;
    return(pos);               //return the change of position
}

//Move tilt servo. Used /w joystick.
int change_tilt(int pos,int t){
    pos=pos+t;                   // Increment/decrement the Servo angle
    if(pos<tilt_limit_1)                  // minimum angle of servo is 0 degree
      pos=tilt_limit_1;
    if (pos>tilt_limit_2)                //  maximum angle of servo is 180 degree
      pos=tilt_limit_2;
    return(pos);               //return the change of position
}

//Play a melody. Accepts a melody and durations for notes as params.
void playMelody(int melody[],int noteDurations[]){
  for (int thisNote = 0; thisNote < 2; thisNote++) {
    // to calculate the note duration, take one second (1000) divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 500 / noteDurations[thisNote];
    tone(2, melody[thisNote], noteDuration);
    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% (1.30) seems to work well:
    int pauseBetweenNotes = noteDuration * 2.00;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(2);
  }
  delay(1000); // to finish song
}

//Move in x_direction in mode 1 or 2.
void x_direction(){
    Serial.println("**********XXXXX**********");
    panErrorOld=panError;
    Serial.print("panErrorOld = :");Serial.println(panErrorOld);
    
    panSetpoint = map(pixy.ccc.blocks[blocknum].m_x,0,315,pan_limit_2,pan_limit_1);
    Serial.print("Setpoint (pixy mapped to +130/+30 deg.:");Serial.println(panSetpoint);
    panInput = float(pan_servo.read());
    Serial.print("Input (servo location in deg.:");Serial.println(panInput);
    panError=panSetpoint-panInput;
    Serial.print("Error in x-direc.:");Serial.println(panError);
    
    panErrorChange=panError-panErrorOld;
    panErrorSlope=panErrorChange/dt;
    panErrorArea=panErrorArea+panError*dt;

    Serial.println(pan_k1*panError);Serial.println(pan_k2*panErrorSlope);Serial.println(pan_k3*panErrorArea);
    Serial.println(pan_k1*panError+pan_k2*panErrorSlope+pan_k3*panErrorArea);
    panServoVal=float(pan_servo.read())+pan_k1*panError+pan_k2*panErrorSlope+pan_k3*panErrorArea; //PDI
    
    Serial.print("ServoVal :");Serial.println(panServoVal);
    panServoVal=constrain(panServoVal,pan_limit_1,pan_limit_2);
    Serial.print("ServoVal after constrain: ");Serial.println(panServoVal);
    if (abs(panError) >0.10){
      pan_servo.write(panServoVal);
    }
}

//Move in y_direction in mode 1 or 2 using PID.
void y_direction(){
    Serial.println("**********YYYYY**********");
//    milliOld=milliNew;
//    milliNew=millis();
//    dt=milliNew-milliOld;
    tiltErrorOld=tiltError;
    //0,207
    tiltSetpoint = map(pixy.ccc.blocks[blocknum].m_y,0,207,tilt_limit_2,tilt_limit_1);
    Serial.print("Setpoint (pixy mapped to 0/+207 deg.:");Serial.println(tiltSetpoint);
    tiltInput = tilt_servo.read();
    Serial.print("Input (servo location in deg.:");Serial.println(tiltInput);
    tiltError=tiltSetpoint-tiltInput;
    Serial.print("Error in y-direc.:");Serial.println(tiltError);
    
    tiltErrorChange=tiltError-tiltErrorOld;
    tiltErrorSlope=tiltErrorChange/dt;
    tiltErrorArea=tiltErrorArea+tiltError*dt;
    
    tiltServoVal=tilt_servo.read()+(tilt_k1*tiltError+tilt_k2*tiltErrorSlope+tilt_k3*tiltErrorArea); //PDI
    Serial.print("ServoVal ");Serial.println(tiltServoVal);
    tiltServoVal=constrain(tiltServoVal,tilt_limit_1,tilt_limit_2);
    if (abs(tiltError) >0.10){
      tilt_servo.write(tiltServoVal);
    }
}

//Sort struct by temperature.
void sort_temps(){
  std::sort(std::begin(light), std::end(light), cmpfunc);
  for(light_type e : light){
 }
}

//COmparator function used to specifically compare temps in the struct.
bool cmpfunc(light_type a, light_type b) { 
  return a.temp > b.temp; 
}

//Record temp and location. Raises turret by an "offset" because the flame
//sensor is below the camera. Allows sensor to record. Lowers it back down after.
void record(int i){
  int offset = 10;
  tilt_servo.write(tilt_servo.read()+offset);
  delay(1000);
  light[i].angle_x=pan_servo.read();
  light[i].angle_y=tilt_servo.read();
  light[i].temp = analogRead(a0Pin);
  Serial.print("Recording. x, y, temp: ");Serial.print(light[i].angle_x);Serial.print(" ");
    Serial.print(light[i].angle_y);Serial.print(" ");Serial.println(light[i].temp);
  delay(1000);
  tilt_servo.write(tilt_servo.read()-offset);
  delay(1000);
}

//Swivels to face highest temp light source. Beeps to signify it's "firing".
void firefighter(){
  Serial.println("In firegihter.*****************************************************************************************************************************");
  Serial.println("Before sort:");
  for (int i = 0; i <3; i++ ){
    Serial.println(i);
    Serial.println(light[i].angle_x);
    Serial.println(light[i].angle_y);
    Serial.println(light[i].temp);
    Serial.println("*****");
  }
  sort_temps();
  Serial.println("After sort:");
  for (int i = 0; i <3; i++ ){
    Serial.println(i);
    Serial.println(light[i].angle_x);
    Serial.println(light[i].angle_y);
    Serial.println(light[i].temp);
    Serial.println("*****");
  }
  //pan_servo.write(light[0].angle_x);
  //tilt_servo.write(light[0].angle_y);
  pan_servo.write(light[0].angle_x);
  tilt_servo.write(light[0].angle_y);
  playMelody(melody1,noteDuration2);
    delay(250);
    //Fire recoil servo and beep
}

//-----Functions related to time
//On a set interval, calls a movement function to move in case 1/2 via PID.
void x_update(){
  if((millis() - lastUpdate) > interval1){ //time to update
    if(blocknum < pixy.ccc.numBlocks){
      lastUpdate = millis();
      x_direction();
    }
  }
}

//On a set interval, calls a movement function to move in case 1/2 via PID.
void y_update(){
  if((millis() - lastUpdate1) > interval1){ //time to update
    if(blocknum < pixy.ccc.numBlocks){
      lastUpdate1 = millis();
      y_direction();
    }
  }
}

//On a set interval, in case 2 increments the blocknum (the light source focused on)
void num_update(){
  if((millis() - lastUpdate2) > 10000){ //time to update
      lastUpdate2 = millis();
      if(blocknum==(pixy.ccc.numBlocks)){
        Serial.println("Max num of blocks reached. Breaking num_update.");
        return;
      }
      else{
        record(blocknum);
        Serial.println(light[blocknum].temp);
        blocknum++;
      }
  }
}

//On a set interval, calls firefighter in case 2.
void fire_update(){
  if((millis() - lastUpdate3) > 21000){ //time to update
      lastUpdate3 = millis();
      firefighter();
      }
  }
