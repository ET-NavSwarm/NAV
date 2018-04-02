#include <Adafruit_GPS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <SparkFunMPL3115A2.h>
#include <IMU.h>
#include <math.h>
#include <pt.h>
#include <clock.h>
#include <timer.h>

// if true debug information will be periodically sent on Serial
#define DEBUG_INFO false

// set any to false to skip that sensor. useful for debugging individual sensors
#define ENABLE_GPS true
#define ENABLE_IMU true
#define ENABLE_PRESSURE true

// directions passed back from irsensor()
#define FORWARD 0
#define BACKWARD 1
#define STOP 2
#define LEFT 3
#define RIGHT 4

// Set CALIBRATE_GYRO to true or false to enable or disable the gyroscope calibration step
// This step takes many samples on initialization and averages them to zero out
#define CALIBRATE_GYRO false

boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


Adafruit_GPS GPS(&Serial1);

float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

IMU imu;

MPL3115A2 pressure_sensor;

// protothread declarations
static struct pt ptSerial;
static struct pt ptSensor;
static struct pt ptDrive;
static struct pt ptDebug;

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


// current data values
float temperature = -1.0;
float pressure = -1.0;
double goalLat = -1.0;
double goalLon = -1.0;
double currLat = -1.0;
double currLon = -1.0;

// simple state for serial control
int forceDirection = FORWARD;

void setup(){
  Serial.begin(9600);               // starts the serial monitor
  
  /* set up motors */
  pinMode(2,OUTPUT);                // PWM pin 1 from motor driver
  pinMode(3,OUTPUT);                // PWM pin 2 from motor driver
  pinMode(22,OUTPUT);               //Direction pin 1 from motor driver
  pinMode(23,OUTPUT);               // Direction pin 2 from motor driver
  digitalWrite(22,HIGH);            // HIGH for forward LOW for reverse
  digitalWrite(23,HIGH);            // HIGH for forward LOW for reverse

  #if USE_PRESSURE
  /* initialize pressure sensor */
  Wire.begin();
  pressure_sensor.begin();
  pressure_sensor.setModeBarometer();
  pressure_sensor.setOversampleRate(7);
  pressure_sensor.enableEventFlags();
  #endif

  #if USE_GPS
  /* initialize GPS */
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  #endif

  #if USE_IMU
  /* initialize IMU */
  // imu.initalize(CALIBRATE_GYRO);
  #endif
  
  PT_INIT(&ptDrive);
  PT_INIT(&ptSerial);
  PT_INIT(&ptSensor);
  PT_INIT(&ptDebug);

  #if USE_GPS
  // initialization of interrupt we'll use to babysit the gps
  // this shares the millis() interrupt, so it runs once every millisecond (as such it needs to be very short)
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  #endif
}

#if(USE_GPS == true)
SIGNAL(TIMER0_COMPA_vect) {
  // this stores the read byte internall in the GPS
  // when a string is eventually complete, GPS.newNMEAreceived() will be true (but only until it is read)
  GPS.read();
}
#endif

static PT_THREAD(serialThread(struct pt *pt)){
  PT_BEGIN(pt);
  static char buff[65]; // one extra character to always null terminate

  while(1){
    PT_WAIT_UNTIL(pt, Serial.available());
    int numBytes = Serial.readBytes(buff, 64);
    // simple echo for now just to show it works
    Serial.println(buff);

    // change force direction if a direction or "auto" is input
    if (!strcmp(buff, "stop")) forceDirection = STOP;
    if (!strcmp(buff, "forward")) forceDirection = FORWARD;
    if (!strcmp(buff, "backward")) forceDirection = BACKWARD;
    if (!strcmp(buff, "left")) forceDirection = LEFT;
    if (!strcmp(buff, "right")) forceDirection = RIGHT;
    if (!strcmp(buff, "auto")) forceDirection = -1;

    // clear buffer
    for (int i=0; i<numBytes; i++) buff[i] = '\0';
  }
  
  PT_END(pt);
}

static PT_THREAD(driveThread(struct pt *pt)){
  static struct timer t_main;
  static struct timer t_movement;
  
  PT_BEGIN(pt);

  timer_set(&t_main, 0.1*CLOCK_SECOND);
  timer_set(&t_movement, 0); // immediately expire for now

  while(1){
    PT_WAIT_UNTIL(pt, timer_expired(&t_main) && timer_expired(&t_movement));
    int dir = irsensor();
    switch (dir) {
      case BACKWARD:
        digitalWrite(22,LOW);            // HIGH for forward LOW for reverse
        digitalWrite(23,LOW);            // HIGH for forward LOW for reverse
        analogWrite(2,power/2);
        analogWrite(3,power/2);
        timer_set(&t_movement, CLOCK_SECOND);
        Serial.println("drive -- backward");
        break;
      case STOP:  // Currently only called in testing
        analogWrite(2,0);
        analogWrite(3,0);
        timer_set(&t_movement, 0.5*CLOCK_SECOND);
        Serial.println("drive -- stop");
        break;
      case LEFT:
        digitalWrite(22,HIGH);
        digitalWrite(23,LOW);
        analogWrite(2,turnpower);
        analogWrite(3,turnpower);
        timer_set(&t_movement, 0.4*CLOCK_SECOND);
        Serial.println("drive -- left");
        break;
      case RIGHT:
        digitalWrite(22,LOW);
        digitalWrite(23,HIGH);
        analogWrite(2,turnpower);
        analogWrite(3,turnpower);
        timer_set(&t_movement, 0.4*CLOCK_SECOND);
        Serial.println("drive -- right");
        break;
      default:
        // FORWARD
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
        analogWrite(2,power);
        analogWrite(3,power);
        // no timer so IR sensors are constantly checked when moving forward
        Serial.println("drive -- forward");
    }
    timer_reset(&t_main);
  }
  PT_END(pt);
}

static PT_THREAD(sensorThread(struct pt *pt)){
  static struct timer t;
  PT_BEGIN(pt);

  timer_set(&t, 0.2*CLOCK_SECOND);
  
  while(1){
    PT_WAIT_UNTIL(pt, timer_expired(&t));
    
    // read, store, and use sensor values
    #if USE_PRESSURE
    pressure = pressure_sensor.readPressure();
    temperature = pressure_sensor.readTemp();
    #endif

    #if USE_GPS
    // update internal gps values and store what we specifically need.
    if (GPS.newNMEAreceived()) {
      GPS.parse(GPS.lastNMEA());
      currLat = GPS.latitude;
      currLon = GPS.longitude;
    }
    #endif

    #if USE_IMU
    // update internal imu values
    imu.read();
    #endif

    Serial.println("sensors updated");
    
    timer_reset(&t);
  }
  PT_END(pt);
}

static PT_THREAD(debugThread(struct pt *pt)){
  static struct timer t;
  PT_BEGIN(pt);

  timer_set(&t, 2*CLOCK_SECOND);

  while(1){
    PT_WAIT_UNTIL(pt, timer_expired(&t));
    
    Serial.println("DEBUG INFO HERE");
    
    timer_reset(&t);
  }
  PT_END(pt);
}

void loop(){
  driveThread(&ptDrive);
  serialThread(&ptSerial);
  sensorThread(&ptSensor);
  if(DEBUG_INFO) debugThread(&ptDebug);
}

void setPreviousPins( int s0, int s1, int s2, int s3 ){
    lastsensorpindata0 = s0;
    lastsensorpindata1 = s1;
    lastsensorpindata2 = s2;
    lastsensorpindata3 = s3;  
}

int irsensor(){
  // testing for *very* simple serial control
  if (forceDirection != -1) {
    return forceDirection;
  }
  
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
    /*else {
      setPreviousPins( distance0, distance1, distance2, distance3);
      return STOP;
    }*/
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
