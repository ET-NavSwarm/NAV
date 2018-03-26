#include <Adafruit_GPS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <SparkFunMPL3115A2.h>
#include <IMU.h>
#include <math.h>
#include "TinyGPS++.h"
#include <pt.h>
#include <clock.h>
#include <timer.h>

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
// Set CALIBRATE_GYRO to true or false to enable or disable the gyroscope calibration step
// This step takes many samples on initialization and averages them to zero out
#define CALIBRATE_GYRO true
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


Adafruit_GPS GPS(&Serial1);
TinyGPSPlus gps;

float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

IMU imu;

MPL3115A2 pressure_sensor;


// protothread declarations
static struct pt ptSerial;
static struct pt ptSensors;
static struct pt ptDrive;


bool needsLocation = true;
bool updated = false;

int sensorpin0 = A0;               
int sensorpin1 = A1;          
int sensorpin2 = A2;
int sensorpin3 = A3; 

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
  /* set up motors */
  pinMode(2,OUTPUT);                // PWM pin 1 from motor driver
  pinMode(3,OUTPUT);                // PWM pin 2 from motor driver
  pinMode(22,OUTPUT);               //Direction pin 1 from motor driver
  pinMode(23,OUTPUT);               // Direction pin 2 from motor driver
  digitalWrite(22,HIGH);            // HIGH for forward LOW for reverse
  digitalWrite(23,HIGH);            // HIGH for forward LOW for reverse
  Serial.begin(9600);               // starts the serial monitor

  /* initialize pressure sensor */
  Wire.begin();
  pressure_sensor.begin();
  pressure_sensor.setModeBarometer();
  pressure_sensor.setOversampleRate(7);
  pressure_sensor.enableEventFlags();

  /* initialize GPS */
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  /* initialize IMU */
  imu.initalize(CALIBRATE_GYRO);

  PT_INIT(&ptSerial);
  PT_INIT(&ptSensors);
  PT_INIT(&ptDrive);
}

static PT_THREAD(serialThread(struct pt *pt)){
  PT_BEGIN(pt);
  static char buff[64];

  while(1){
    PT_WAIT_UNTIL(pt, Serial.available());
    Serial.readBytes(buff, 64);
    // simple echo for now just to show it works
    Serial.println(buff);
  }
  
  PT_END(pt);
}

static PT_THREAD(sensorThread(struct pt *pt)){
  static struct timer t;
  PT_BEGIN(pt);

  timer_set(&t, 0.1*CLOCK_SECOND); // run 10 times per second
  
  while(1){
    PT_WAIT_UNTIL(pt, timer_expired(&t));
    
    // read, store, and use sensor values
    
    timer_reset(&t);
  }
  PT_END(pt);
}

static PT_THREAD(driveThread(struct pt *pt)){
  static struct timer t;
  PT_BEGIN(pt);

  timer_set(&t, 0.1*CLOCK_SECOND); // run 10 times per second

  while(1){
    PT_WAIT_UNTIL(pt, timer_expired(&t));
  }
  PT_END(pt);
}


int counter = 0;

void loop(){
  //if ( counter-- <= 0 ){
  //  adjust(NORTH, 0 );
  //  counter = 8000;
  //}
  //update gps
  //reportGPS();
  
  //Serial.println(getHeading());
  
  //delay(1000);
  //move( irsensor() );


  serialThread(&ptSerial);
}




void moveForward(){
    digitalWrite(22,HIGH);            // HIGH for forward LOW for reverse
    digitalWrite(23,HIGH);            // HIGH for forward LOW for reverse
    analogWrite(2,power);
    analogWrite(3,power);
    delay(1000);  
}



int irsensor(){
  //Distance (cm) = 4800/(SensorValue - 20)
  int distance0 = analogRead(sensorpin0);
  int distance1 = analogRead(sensorpin1); 
  int distance2 = analogRead(sensorpin2);
  int distance3 = analogRead(sensorpin3);
  
  // something is way to close to the front of bot
  if ( distance0 > 300 || distance1 > 300  ){ 
    if ( lastsensorpindata0 > 300 || lastsensorpindata1 > 300 ){
      setPreviousPins( 0, 0, 0, 0);
      return BACKWARD;
    }
    else {
      //setPreviousPins( distance0, distance1, distance2, distance3);
      //return STOP;
    }
  } 

  // obstacle in front
  else if ( distance0 > 210 || distance1 > 210 ) {
    if ( lastsensorpindata0 > 210 || lastsensorpindata1 > 210 ){
      setPreviousPins( 0, 0, 0, 0); 
      if ( distance2 > distance3 ){
        return RIGHT;
      }
      else {
        return LEFT;
      }
    } 
  }

 // obstacle to the sides
  else if ( distance2 > 300 || distance3 > 300 ) {
    if ( lastsensorpindata2 > 300 || lastsensorpindata3 > 300 ){
      setPreviousPins( 0, 0, 0, 0); 
      if ( distance2 > distance3 ){
        return RIGHT;
      }
      else {
        return LEFT;
      }
    } 
  }
  setPreviousPins( distance0, distance1, distance2, distance3);
  return FORWARD;
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


float getBearing(double lat, double lon, double det_lat, double det_lon){
  // get bearing based on two GPS locations
  double y = sin(det_lon - lon) * cos(det_lat);
  double x = cos(lat)*sin(det_lat) - sin(lat)*cos(det_lat)*cos(det_lon - lon);
  return atan2(y, x);
}

float getHeading(){
  // since we use the z axis for heading, and azimuth is calculated clockwise:
  return -atan2(imu.mag_y(),imu.mag_x());
}

// For imu heading direction testing
void adjust( int goal, int displacement){
  bool searching = true;
  while( searching ){
     searching = adjustHelper( goal, displacement);
  }
}


bool adjustHelper( int goal, int displacement ){
  /* update the IMU internal values, accessed through methods for each variable */
  imu.read();
  /* Calculate the heading using the magnetometer */
  int heading = getHeading();
  Serial.print(F("Heading: "));
  Serial.print(heading);
  Serial.print(F("; "));


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
  
  Serial.println(F(""));
}
 




void setPreviousPins( int s0, int s1, int s2, int s3 ){
    lastsensorpindata0 = s0;
    lastsensorpindata1 = s1;
    lastsensorpindata2 = s2;
    lastsensorpindata3 = s3;  
}

void reportPressure(){
  Serial.print("Pressure(Pa):");
  Serial.print(pressure_sensor.readPressure(), 2);
  Serial.print(" Temp(f):");
  Serial.print(pressure_sensor.readTempF(), 2);
  Serial.println();
}

void reportGPS(){
  if (GPS.newNMEAreceived()) {
    String NMEA = GPS.lastNMEA();
    Serial.println(NMEA);
    for(int i=0; i<NMEA.length(); i++)
      gps.encode(NMEA[i]);
    Serial.println(gps.satellites.value());
    Serial.println(gps.location.lat(), 6);
    Serial.println(gps.location.lng(), 6);
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }
}


