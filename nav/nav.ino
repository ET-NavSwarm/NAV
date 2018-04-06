#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <SparkFunMPL3115A2.h>
#include <math.h>
#include <pt.h>
#include <timer.h>

/* ##### +Behavior changes ##### */
// simple state for serial control
int forceDirection = -1;

//This value is the power sent to the robot.
//0 is no power and 255 is max power. 
#define POWER 80
#define TURNPOWER 120

//This is the rate the bot will adjust toward the goal * CLOCK_SECOND
int adjustRate = 1;
/* ##### -Behavior changes ##### */

/* ##### +Debugging ##### */
// frequency to send large amount of debug information on Serial
// set to 0 to disable
#define DEBUG_INFO 1

// set any to false to skip that sensor. useful for debugging individual sensors
#define USE_GPS false
#define USE_IMU true
#define USE_PRESSURE false

// echoes raw NMEA data to Serial
#define ECHO_GPS false
// use with ECHO_GPS to check antenna status. (you won't see anything without ECHO_GPS)
// look for $PGTOP,11,X*## in the Serial Monitor. X=3: external antenna, X=2: internal antenna
#define DEBUG_ANTENNA false

// Causes a message to be sent on Serial whenever a thread is yielded to
#define DEBUG_THREADS true
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
Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);
MPL3115A2 pressure_sensor;

// protothreads
static struct pt ptSerial;
static struct pt ptSensor;
static struct pt ptDrive;
static struct pt ptDebug;
static struct pt ptAdjust;

// used in irsensor()
int lastsensorpindata0 = 0;
int lastsensorpindata1 = 0;
int lastsensorpindata2 = 0;
int lastsensorpindata3 = 0;

// current data values
sensors_vec_t orientation = {};
float currAltitude = -1.0;
float goalLat = -1.0;
float goalLon = -1.0;
float goalBearing = -1.0;
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

  #if USE_PRESSURE || USE_IMU
    Wire.begin(); // join i2c bus
  #endif
  
  #if USE_PRESSURE
    /* initialize pressure sensor */
    pressure_sensor.begin();
    pressure_sensor.setModeAltimeter();
    pressure_sensor.setOversampleRate(7);
    pressure_sensor.enableEventFlags();
  #endif

  #if USE_IMU
    /* initialize IMU */
    if(!mag.begin() || !accel.begin()){
      Serial.println("IMU connection error");
      while(1);
    }
  #endif

  // initialize threads
  PT_INIT(&ptDrive);
  PT_INIT(&ptSerial);
  PT_INIT(&ptSensor);
  PT_INIT(&ptDebug);
  PT_INIT(&ptAdjust);

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

  // debug hardcoded goal
  goalLat = 43.14063924666028;
  goalLon = -70.94233550243678;
  goalBearing = getBearingTo(goalLat, goalLon);
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
  static struct timer t_movement;
  static struct timer t_no_adjust;
  
  PT_BEGIN(pt);
  
  timer_set(&t_movement, 0); // immediately expire for now
  timer_set(&t_no_adjust, 0); // immediately expire for now

  while(1){
    PT_WAIT_UNTIL(pt, timer_expired(&t_movement));
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
        analogWrite(2,TURNPOWER/2);
        analogWrite(3,TURNPOWER/2);
        timer_set(&t_movement, 0.6*CLOCK_SECOND);
        timer_set(&t_no_adjust, 1.4*CLOCK_SECOND);
        #if DEBUG_THREADS
          Serial.println("drive -- left");
        #endif
        break;
      case RIGHT:
        digitalWrite(22,LOW);
        digitalWrite(23,HIGH);
        analogWrite(2,TURNPOWER/2);
        analogWrite(3,TURNPOWER/2);
        timer_set(&t_movement, 0.6*CLOCK_SECOND);
        timer_set(&t_no_adjust, 1.4*CLOCK_SECOND);
        #if DEBUG_THREADS
          Serial.println("drive -- right");
        #endif
        break;
      default:
        if(timer_expired(&t_no_adjust)){
          adjustGoal();
        } else {
          //FORWARD
          digitalWrite(22,HIGH);
          digitalWrite(23,HIGH);
          analogWrite(2,POWER);
          analogWrite(3,POWER);
          #if DEBUG_THREADS
            Serial.println("drive -- forward");
          #endif
        }
        // no timer so IR sensors are constantly checked when moving forward
    }
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
          goalBearing = getBearingTo(goalLat, goalLon);
        }
      }
    #endif

    #if USE_IMU
      // update orientation struct
      sensors_event_t mag_event;
      mag.getEvent(&mag_event);
      // TODO: There is a fucntion of the lib to correct for tilt
      dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);
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

    #if USE_GPS
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
    #endif

    #if USE_IMU
      Serial.print("Heading: ");
      Serial.println(orientation.heading);
    #endif
    
    timer_reset(&t);
  }
  PT_END(pt);
}

static PT_THREAD(adjustThread(struct pt *pt)){
  static struct timer t;
  PT_BEGIN(pt);

  timer_set(&t, adjustRate*CLOCK_SECOND);

  while(1){
    PT_WAIT_UNTIL(pt, timer_expired(&t));
    adjustGoal();
    timer_set(&t, adjustRate*CLOCK_SECOND);
  }
  PT_END(pt);
}

void loop(){
  driveThread(&ptDrive);
  serialThread(&ptSerial);
  sensorThread(&ptSensor);
  //adjustThread(&ptAdjust);
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
      adjustRate = 5;
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
      adjustRate = 5;
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
      adjustRate = 5;
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
  adjustRate = 1;
  return FORWARD;
}

float getBearingTo(float det_lat, float det_lon){
  // bearing is flipped to counterclockwise to match heading
  return calcBearing(currLat, currLon, det_lat, det_lon);
}

float calcBearing(float start_lat, float start_lon, float det_lat, float det_lon){
//  get bearing based on two GPS locations - DEGREES
  float y = sin(det_lon - start_lon) * cos(det_lat);
  float x = cos(start_lat)*sin(det_lat) - sin(start_lat)*cos(det_lat)*cos(det_lon - start_lon);
  float theta = atan2(y, x);
  float deg = theta*(180/M_PI);

  if(deg < 0) deg = 360 + deg;
  return deg;

}

int adjustGoal(){
  int dir = -1;

  if(orientation.heading - goalBearing > 5 || orientation.heading - goalBearing < -5){

  // update orientation struct
      sensors_event_t mag_event;
      mag.getEvent(&mag_event);
      // TODO: There is a fucntion of the lib to correct for tilt
      dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);

    float diff = goalBearing - orientation.heading;
    if(diff < 0)
      diff += 360;
    if(diff > 180)
      dir = LEFT;
    else
      dir = RIGHT;
    
    Serial.println(goalBearing - orientation.heading);
    if(dir == LEFT){
      // LEFT
      digitalWrite(22,HIGH);
      digitalWrite(23,LOW);
      analogWrite(2,60);
      analogWrite(3,60);
      #if DEBUG_THREADS
        Serial.println("adjust -- left");
      #endif
    }
    else{
      // RIGHT
      digitalWrite(22,LOW);
      digitalWrite(23,HIGH);
      analogWrite(2,60);
      analogWrite(3,60);
      #if DEBUG_THREADS
        Serial.println("adjust -- right");
      #endif
    } 
  } else {
    // Done Adjusting, move FORWARD
    digitalWrite(22,HIGH);
    digitalWrite(23,HIGH);
    analogWrite(2,POWER);
    analogWrite(3,POWER);
    #if DEBUG_THREADS
      Serial.println("to goal -- forward");
    #endif
  }
}
