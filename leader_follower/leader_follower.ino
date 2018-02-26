#include <Adafruit_GPS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <SoftwareSerial.h>
#include "SparkFunMPL3115A2.h"
#include <Math.h>


#define FORWARD 0
#define BACKWARD 1
#define STOP 2
#define LEFT 3
#define RIGHT 4

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

#define EMPTY 0
#define OCCUPIED 1
#define BOT 2
#define GOAL 3


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


Adafruit_GPS GPS(&Serial1);
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_vec_t   orientation;


MPL3115A2 pressure_sensor;


bool needsLocation = true;
bool updated = false;

int sensorpin0 = A3; //front right              
int sensorpin1 = A0; //front left working intermittently        
int sensorpin2 = A2; //right
int sensorpin3 = A1; //left 

int lastsensorpindata0 = 0;
int lastsensorpindata1 = 0;
int lastsensorpindata2 = 0;
int lastsensorpindata3 = 0;
int previousheading = 0;

//This value is the power sent to the robot.
//0 is no power and 255 is max power. 
const int power = 80; 
const int turnpower = 120;


void setup(){
  pinMode(2,OUTPUT);                // PWM pin 1 from motor driver
  pinMode(3,OUTPUT);                // PWM pin 2 from motor driver
  pinMode(22,OUTPUT);               // Direction pin 1 from motor driver
  pinMode(23,OUTPUT);               // Direction pin 2 from motor driver
  digitalWrite(22,HIGH);            // HIGH for forward LOW for reverse
  digitalWrite(23,HIGH);            // HIGH for forward LOW for reverse
  Serial.begin(9600);               // starts the serial monitor
 // Serial.begin(115200);
  initSensors();
  //  initGPS();
  delay(500); 
}



int counter = 0;

void loop(){ 

  //if ( counter-- <= 0 ){
  //  adjust(NORTH, 0 );
  //  counter = 8000;
  //}
  //reportPressure();
  move( irsensor() );
}




void moveForward(){
    digitalWrite(22,HIGH);            // HIGH for forward LOW for reverse
    digitalWrite(23,HIGH);            // HIGH for forward LOW for reverse
    analogWrite(2,power);
    analogWrite(3,power);
    delay(1000);  
}



int irsensor(){
//  //Distance (cm) = 4800/(SensorValue - 20)
  int distance0 = analogRead(sensorpin0); // front right
  int distance1 = analogRead(sensorpin1); // front left
  int distance2 = analogRead(sensorpin2); // right
  int distance3 = analogRead(sensorpin3); // left
//
//  //there is something in front of the bot, follow
//  if(distance0 > 300 && distance1 > 300 && distance0 < 100 && distance1 < 100){
//    return FORWARD;
//  }
//
//  //there is nothing in front of the bot, stop
//  else{
//    return STOP;
//  }
//    //stop
    
//  // something is too close to the front of bot
//  if ( distance0 > 300 || distance1 > 300  ){ 
//    if ( lastsensorpindata0 > 300 || lastsensorpindata1 > 300 ){
//      setPreviousPins( 0, 0, 0, 0);
//      return STOP;
//    }
//    else {
//      //setPreviousPins( distance0, distance1, distance2, distance3);
//      //return STOP;
//    }
//  } 

//  // obstacle in front
//  else if ( distance0 > 210 || distance1 > 210 ) {
//    if ( lastsensorpindata0 > 210 || lastsensorpindata1 > 210 ){
//      setPreviousPins( 0, 0, 0, 0); 
//      if ( distance2 > distance3 ){
//        return RIGHT;
//      }
//      else {
//        return LEFT;
//      }
//    } 
//  }

 //delay(1000); //IR debugging delay
 Serial.print("Right sensor: ");
 Serial.print(distance2);
 Serial.print("\n");
 Serial.print("Left sensor: ");
 Serial.print(distance3);
 Serial.print("\n");
 Serial.print("Front Right sensor: ");
 Serial.print(distance0);
 Serial.print("\n");
 Serial.print("Front Left sensor: ");
 Serial.print(distance1);
 Serial.print("\n");
 
 // bot moved to the sides
  if ( distance2 > 300 || distance3 > 300) {
      if ( distance2 > distance3 ){
        return LEFT;
      }
      else {
        return RIGHT;
      }
  }

  //follow the bot
  else{
    if(distance0 > 230 && distance0 < 400){
      return FORWARD;
    }
    else if ( lastsensorpindata0 > 210 ) return FORWARD;
    else return STOP;
  }
  
  setPreviousPins( distance0, distance1, distance2, distance3);
  return STOP;
}





void move(int option){
  //forward 

  if(option == FORWARD){
    digitalWrite(22,HIGH);            // HIGH for forward LOW for reverse
    digitalWrite(23,HIGH);            // HIGH for forward LOW for reverse
    analogWrite(2,power);
    analogWrite(3,power);
    return;
  } 
  //backwards
  else if(option == BACKWARD){
    digitalWrite(22,LOW);   
    digitalWrite(23,LOW); 
    analogWrite(2,50);       //forward
    analogWrite(3,50);   //reverse 
    delay(1000);
    return;
  }
  //Stall Stop 
  else if(option == STOP){
    analogWrite(2,0);
    analogWrite(3,0);
    delay(100);
    return;
  } 
   
  //turn left
  if(option == LEFT){ 
    digitalWrite(22,HIGH);   
    digitalWrite(23,LOW); 
    analogWrite(2,turnpower);       //forward
    analogWrite(3,turnpower);   //reverse 
    delay(400);
  }
  //turn right
  else if(option == RIGHT){     
    digitalWrite(22,LOW);
    digitalWrite(23,HIGH); 
    analogWrite(2,turnpower);       //forward
    analogWrite(3,turnpower);   //reverse 
    delay(400);  
  }
}








// For imu heading direction testing
void adjust( int goal, int displacement){
  bool searching = true;
  while( searching ){
     Serial.println("      stuck1");
     searching = adjustHelper( goal, displacement);
  }
}
    


bool adjustHelper( int goal, int displacement ){
  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  Serial.println("      stuck2");
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */
    Serial.print(F("Heading: "));
    Serial.print(orientation.heading);
    Serial.print(F("; "));
    int heading = orientation.heading;


    if ( goal == NORTH ) {
      int left = displacement+344;
      int right = displacement+15;
      int turnthreshhold = displacement+180;
      if ( heading > left || heading < right ){
          move(STOP);
          return false;
      }
      else if ( heading < turnthreshhold )
          move(LEFT);
      else 
         move(RIGHT); 
      return true;  
    }


     if ( goal == SOUTH ) {
      int right = displacement+205;
      int left = displacement+155;
      int turnthreshhold = displacement + 180;
      if ( heading > left && heading < right ){
          move(STOP);
          return false;
      }
      else if( (heading < turnthreshhold) )
          move(RIGHT);
      else 
         move(LEFT); 
      return true;  
    }


    if ( goal == EAST ){
      int left = displacement+65;
      int right = displacement+115;
      int turnthreshhold = displacement+270;
      if ( heading > left && heading < right ){
          move(STOP);
          return false;
      }
      else if ( (heading < turnthreshhold)  &&  ( heading > left) )  
        move(LEFT);
      else 
        move(RIGHT); 
      return true;       
    }


    if ( goal == WEST ){
      int left = displacement+245;
      int right = displacement+295;
      int turnthreshhold = displacement+90;      
      if ( heading > left && heading < right ){
          return false;
      }
      else if ( (heading > turnthreshhold)  &&  ( heading < left) )  
        move(RIGHT);
      else 
        move(LEFT); 
      return true;       
    }
  }
  Serial.println(F(""));
}
 




void setPreviousPins( int s0, int s1, int s2, int s3 ){
    lastsensorpindata0 = s0;
    lastsensorpindata1 = s1;
    lastsensorpindata2 = s2;
    lastsensorpindata3 = s3;  
}

/*
void initGPS(){

  
}
*/

void reportPressure(){
  Serial.print("Pressure(Pa):");
  Serial.print(pressure_sensor.readPressure(), 2);
  Serial.print(" Temp(f):");
  Serial.print(pressure_sensor.readTempF(), 2);
  Serial.println();
}

void initSensors(){
//  if(!accel.begin())
//  {
//    /* There was a problem detecting the LSM303 ... check your connections */
//    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
//    while(1);
//  }
//  if(!mag.begin())
//  {
//    /* There was a problem detecting the LSM303 ... check your connections */
//    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
//    while(1);
//  }
  Wire.begin();
  pressure_sensor.begin();
  pressure_sensor.setModeBarometer();
  pressure_sensor.setOversampleRate(7);
  pressure_sensor.enableEventFlags();
}



