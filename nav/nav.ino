#include <Adafruit_GPS.h>
#include <SparkFunMPL3115A2.h>
#include <IMU.h>
#include <math.h>
#include <pt.h>
#include <timer.h>

/* ##### +Behavior changes ##### */
// Set CALIBRATE_GYRO to true or false to enable or disable the gyroscope calibration step
// This step takes many samples on initialization and averages them to zero out
#define CALIBRATE_GYRO false

// simple state for serial control
int forceDirection = 0;

//This value is the power sent to the robot.
//0 is no power and 255 is max power. 
#define POWER 80
#define TURNPOWER 120
/* ##### -Behavior changes ##### */

/* ##### +Debugging ##### */
// frequency to send large amount of debug information on Serial
// set to 0 to disable
#define DEBUG_INFO 0

// set any to false to skip that sensor. useful for debugging individual sensors
#define USE_GPS true
#define USE_IMU true
#define USE_PRESSURE true

// echoes raw NMEA data to Serial
#define ECHO_GPS false
// use with ECHO_GPS to check antenna status. (you won't see anything without ECHO_GPS)
// look for $PGTOP,11,X*## in the Serial Monitor. X=3: external antenna, X=2: internal antenna
#define DEBUG_ANTENNA false

// Causes a message to be sent on Serial whenever a thread is yielded to
#define DEBUG_THREADS false
/* ##### -Debugging ##### */

/* ##### +Constants ##### */
// directions passed back from irsensor()
#define FORWARD 0
#define BACKWARD 1
#define STOP 2
#define LEFT 3
#define RIGHT 4
/* ##### -Constants ##### */

/* ##### +Declarations ##### */
// Sensors
Adafruit_GPS GPS(&Serial1);
IMU imu;
MPL3115A2 pressure_sensor;

// protothreads
static struct pt ptSerial;
static struct pt ptSensor;
static struct pt ptDrive;
static struct pt ptDebug;

// used in irsensor()
int lastsensorpindata0 = 0;
int lastsensorpindata1 = 0;
int lastsensorpindata2 = 0;
int lastsensorpindata3 = 0;

// current data values
float heading = -1.0;
float currAltitude = -1.0;
float goalLat = -1.0;
float goalLon = -1.0;
float currLat = -1.0;
float currLon = -1.0;
/* ##### -Declarations ##### */

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
    Wire.begin(); // join ic2 bus
    pressure_sensor.begin();
    pressure_sensor.setModeAltimeter();
    pressure_sensor.setOversampleRate(7);
    pressure_sensor.enableEventFlags();
  #endif

  #if USE_IMU
    /* initialize IMU */
    imu.initalize(CALIBRATE_GYRO);
  #endif

  // initialize threads
  PT_INIT(&ptDrive);
  PT_INIT(&ptSerial);
  PT_INIT(&ptSensor);
  PT_INIT(&ptDebug);

  #if USE_GPS
    /* initialize GPS */
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
    #if DEBUG_ANTENNA
      GPS.sendCommand(PGCMD_ANTENNA);
    #else
      GPS.sendCommand(PGCMD_NOANTENNA);
    #endif
    
    // initialization of interrupt we'll use to babysit the gps
    // this shares the millis() interrupt, so it runs once every millisecond (as such it needs to be very short)
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
  #endif
}

#if USE_GPS
  SIGNAL(TIMER0_COMPA_vect) {
    // this stores the read byte internally in the GPS
    // when a string is eventually complete, GPS.newNMEAreceived() will be true (but only until it is read)
    #if ECHO_GPS
      char c = GPS.read();
      if(c) UDR0 = c;
    #else
      GPS.read();
    #endif
  }
#endif

static PT_THREAD(serialThread(struct pt *pt)){
  PT_BEGIN(pt);
  static char buff[65]; // one extra character to always null terminate

  while(1){
    PT_WAIT_UNTIL(pt, Serial.available());
    int numBytes = Serial.readBytes(buff, 64);

    #if DEBUG_THREADS
      // simple echo for now just to show it works
      Serial.println(buff);
    #endif

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
        analogWrite(2,POWER/2);
        analogWrite(3,POWER/2);
        timer_set(&t_movement, CLOCK_SECOND);
        #if DEBUG_THREADS
          Serial.println("drive -- backward");
        #endif
        break;
      case STOP:  // Currently only called in testing
        analogWrite(2,0);
        analogWrite(3,0);
        timer_set(&t_movement, 0.5*CLOCK_SECOND);
        #if DEBUG_THREADS
          Serial.println("drive -- stop");
        #endif
        break;
      case LEFT:
        digitalWrite(22,HIGH);
        digitalWrite(23,LOW);
        analogWrite(2,TURNPOWER);
        analogWrite(3,TURNPOWER);
        timer_set(&t_movement, 0.4*CLOCK_SECOND);
        #if DEBUG_THREADS
          Serial.println("drive -- left");
        #endif
        break;
      case RIGHT:
        digitalWrite(22,LOW);
        digitalWrite(23,HIGH);
        analogWrite(2,TURNPOWER);
        analogWrite(3,TURNPOWER);
        timer_set(&t_movement, 0.4*CLOCK_SECOND);
        #if DEBUG_THREADS
          Serial.println("drive -- right");
        #endif
        break;
      default:
        // FORWARD
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
        analogWrite(2,POWER);
        analogWrite(3,POWER);
        // no timer so IR sensors are constantly checked when moving forward
        #if DEBUG_THREADS
          Serial.println("drive -- forward");
        #endif
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
    
    /* read, store, and use sensor values */
    
    #if USE_PRESSURE
      // store current altitude
      currAltitude = pressure_sensor.readAltitude(); // meters above sea level
    #endif

    #if USE_GPS
      // update internal gps values and store what we specifically need.
      if (GPS.newNMEAreceived()) {
        GPS.parse(GPS.lastNMEA());
        if (GPS.fix) {
          currLat = GPS.latitudeDegrees;
          currLon = GPS.longitudeDegrees;
        }
      }
    #endif

    #if USE_IMU
      // update internal imu values
      imu.read();
    #endif

    #if DEBUG_THREADS
      Serial.println("sensors updated");
    #endif
    
    timer_reset(&t);
  }
  PT_END(pt);
}

static PT_THREAD(debugThread(struct pt *pt)){
  static struct timer t;
  PT_BEGIN(pt);

  timer_set(&t, DEBUG_INFO*CLOCK_SECOND);

  while(1){
    PT_WAIT_UNTIL(pt, timer_expired(&t));
    
      Serial.print("Fix: "); Serial.print((int)GPS.fix);
      Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      if (GPS.fix) {
        Serial.print("Location ");
        Serial.print(GPS.latitudeDegrees, 4);
        Serial.print(", "); 
        Serial.println(GPS.longitudeDegrees, 4);
      } else {
        Serial.println("no fix yet");
      }
    
    timer_reset(&t);
  }
  PT_END(pt);
}

void loop(){
  driveThread(&ptDrive);
  serialThread(&ptSerial);
  sensorThread(&ptSensor);
  #if DEBUG_INFO > 0
    debugThread(&ptDebug);
  #endif
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
  int distance0 = analogRead(A0);
  int distance1 = analogRead(A1); 
  int distance2 = analogRead(A2);
  int distance3 = analogRead(A3);
  
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

float getBearingTo(float det_lat, float det_lon){
  return calcBearing(currLat, currLon, det_lat, det_lon);
}

float calcBearing(float start_lat, float start_lon, float det_lat, float det_lon){
  // get bearing based on two GPS locations
  float y = sin(det_lon - start_lon) * cos(det_lat);
  float x = cos(start_lat)*sin(det_lat) - sin(start_lat)*cos(det_lat)*cos(det_lon - start_lon);
  return atan2(y, x);
}

float getHeading(){
  // since we use the z axis for heading, and azimuth is calculated clockwise:
  return -atan2(imu.mag_y(),imu.mag_x());
}
