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

// This value is the power sent to the robot.
// 0 is no power and 255 is max power. 
#define POWER 80
#define TURNPOWER 120

// Time certain drive actions will be performed. Multiples of seconds.
// Move is for moving after a turn (so the bot doesn't repeatedly turn back and forth)
#define TIME_BACK 1.0
#define TIME_STOP 0.5
#define TIME_TURN 0.6
#define TIME_MOVE (1.0 + TIME_TURN)

// Thresholds for ir sensors. higher=closer
#define IR_THRESH_NEAR 300
#define IR_THRESH_FAR 210

// Rate at which sensors data will be checked
// GPS is excluded. GPS is being constantly read by a millisecond interrupt. It will get new data once per second as it is currently set.
#define SENSOR_RATE 0.2
//Rate at which the bot will make movement decisions. Should be kept low, but too low starves other threads.
#define DRIVE_RATE 0.05

// When adjusting to goal, this is the allowed error on either side in degrees.
#define HEADING_PRECISION 5.0
/* ##### -Behavior changes ##### */

/* ##### +Debugging ##### */
// Most debug info should be disabled unless needed. Timing will be more precise with less time chruning through serial data.

// frequency to send large amount of debug information on Serial
// set to 0 to disable
#define DEBUG_INFO 2

// set any to false to skip that sensor. useful for debugging individual sensors
#define USE_GPS false
#define USE_IMU false
#define USE_PRESSURE false

// echoes raw NMEA data to Serial
#define ECHO_GPS false
// use with ECHO_GPS to check antenna status. (you won't see anything without ECHO_GPS)
// look for $PGTOP,11,X*## in the Serial Monitor. X=3: external antenna, X=2: internal antenna
#define DEBUG_ANTENNA false

// Causes a message to be sent on Serial whenever a thread is yielded to
#define DEBUG_THREADS false

// Send args back
#define DEBUG_ARGS true
/* ##### -Debugging ##### */

/* ##### +Constants ##### */
// directions passed back from irsensor()
#define FORWARD 0
#define BACKWARD 1
#define STOP 2
#define LEFT 3
#define RIGHT 4

// bot states/modes
#define WAITING -1
#define NAVIGATION 0
#define FOLLOW 1
#define MANUAL 2

// headers/opcodes sent to pi
// OPS for OP Send1
#define OPS_RECEIVED '\1'
#define OPS_ERROR '\2'
#define OPS_DEBUG '\3'
#define OPS_STATUS '\4'
#define OPS_GPS '\5'

// headers/opcodes received from pi
// OPR for OP Receive
#define OPR_GPS '\1'
#define OPR_CONTROL '\2'
#define OPR_FORCE_DIRECTION '\3'
#define OPR_MODE '\4'

// classes to send data across serial
// base class with method to serialize and send data
class BASE {
  public:
  void send (){
    Serial.println((char *)(this));
  }
};
class S_RECEIVED: public BASE {
  public:
  const char op = OPS_RECEIVED;
  const char null = '\0';
};
class S_ERROR: public BASE {
  public:
  const char op = OPS_ERROR;
  char msg[62];  // two less than buffer size to fit opcode and terminating /n
  S_ERROR (char* s) {strcpy(msg, s);}
};
class S_DEBUG: public BASE {
  public:
  const char op = OPS_DEBUG;
  char msg[62];  // two less than buffer size to fit opcode and terminating /n
  S_DEBUG (char* s) {strcpy(msg, s);}
};
class S_STATUS: public BASE {
  public:
  const char op = OPS_STATUS;
  char msg[62];  // two less than buffer size to fit opcode and terminating /n
  S_STATUS (char* s) {strcpy(msg, s);}
};
class S_GPS: public BASE {
  public:
  const char op = OPS_GPS;
  float lat,lon;
  const char null = '\0';
  S_GPS (float la, float lo){lat=la; lon=lo;}
};

// structs to interpret incoming data
struct R_GPS {
  float lat;
  float lon;
};
struct R_CONTROL {
  float left;
  float right;
  float duration;
};
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

// bot behavior state
int state = WAITING;

// manual drive direction
float manual_l = 0;
float manual_r = 0;
float manual_d = 0;
/* ##### -Declarations ##### */

void report(char op, const char* s){
  Serial.print(op);
  Serial.println(s);
}

void setup(){
  Serial.begin(115200);  // USB serial

  report(OPS_DEBUG,"Power on");

  #if USE_PRESSURE || USE_IMU
    Wire.begin(); // join i2c bus
  #endif
  
  #if USE_PRESSURE
    /* initialize pressure sensor */
    if(pressure_sensor.begin()){
      pressure_sensor.setModeAltimeter();
      pressure_sensor.setOversampleRate(7);
      pressure_sensor.enableEventFlags();
      report(OPS_DEBUG, "Pressure sensor connected");
    } else {
      report(OPS_ERROR, "Pressure sensor connection error");
      while(1);
    }
  #endif

  #if USE_IMU
    /* initialize IMU */
    if(!mag.begin() || !accel.begin()){
      report(OPS_ERROR, "IMU connection error");
      while(1);
    } else {
      report(OPS_DEBUG, "IMU connecteed");
    }
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

    report(OPS_DEBUG, "GPS setup");
  #endif
  
  /* set up motors */
  pinMode(2,OUTPUT);                // PWM pin 1 from motor driver
  pinMode(3,OUTPUT);                // PWM pin 2 from motor driver
  pinMode(22,OUTPUT);               //Direction pin 1 from motor driver
  pinMode(23,OUTPUT);               // Direction pin 2 from motor driver
  digitalWrite(22,HIGH);            // HIGH for forward LOW for reverse
  digitalWrite(23,HIGH);            // HIGH for forward LOW for reverse
  analogWrite(2,0);
  analogWrite(3,0);
  report(OPS_DEBUG, "Motor pins set");

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
      report(OPS_DEBUG, buff);
    #endif

    if (buff[0] == OPR_GPS){
      // handle interpretting GPS coordinate from buff
      struct R_GPS *args = (struct R_GPS *)(buff+1);
      goalLat = args->lat;
      goalLon = args->lon;
      
      #if DEBUG_ARGS
        Serial.print(OPS_DEBUG); Serial.print("GPS: "); Serial.print(args->lat, 6); Serial.print(", "); Serial.println(args->lon, 6);
      #endif
    } else
    
    if (buff[0] == OPR_CONTROL){
      state = MANUAL;
      
      // handle manual control
      struct R_CONTROL *args = (struct R_CONTROL *)(buff+1);
      manual_l = args->left;
      manual_r = args->right;
      manual_d = args->duration;
      
      #if DEBUG_ARGS
        Serial.print(OPS_DEBUG); Serial.print("CONTROL: "); Serial.print(args->left); Serial.print(", "); Serial.println(args->right);
      #endif
    } else
    
    if (buff[0] == OPR_FORCE_DIRECTION){
      state = NAVIGATION;
      
      // change force direction if a direction or "auto" is input
      if (!strcmp(buff+1, "stop")) forceDirection = STOP;
      if (!strcmp(buff+1, "forward")) forceDirection = FORWARD;
      if (!strcmp(buff+1, "backward")) forceDirection = BACKWARD;
      if (!strcmp(buff+1, "left")) forceDirection = LEFT;
      if (!strcmp(buff+1, "right")) forceDirection = RIGHT;
      if (!strcmp(buff+1, "auto")) forceDirection = -1;

      #if DEBUG_ARGS
        Serial.print(OPS_DEBUG); Serial.println(buff+1);
      #endif
    } else

    if (buff[0] == OPR_MODE){
      // change mode
      #if DEBUG_ARGS
        Serial.print(OPS_DEBUG); Serial.print("Mode change: "); Serial.println(state);
      #endif
      state = (int)buff[1];
    }

    // clear buffer
    for (int i=0; i<numBytes; i++) buff[i] = '\0';

    // mark ready for new command. pass nothing for now. could return an error or nonce later.
    Serial.println(OPS_RECEIVED);
  }
  
  PT_END(pt);
}

static PT_THREAD(driveThread(struct pt *pt)){
  static struct timer t_movement;
  static struct timer t_no_adjust;
  static struct timer t_main;
  
  PT_BEGIN(pt);

  timer_set(&t_main, DRIVE_RATE*CLOCK_SECOND); // timer to limit max drive changes, even in adjust and IR checking
  timer_set(&t_movement, 0); // immediately expire for now
  timer_set(&t_no_adjust, 0); // immediately expire for now

  while(1){
    // we bypass on manual so interrupts are possible.
    PT_WAIT_UNTIL(pt, timer_expired(&t_main) && (state==MANUAL || timer_expired(&t_movement)));

    // behavior state switch
    if (state == WAITING) {
      // stop everything
      analogWrite(2,0);
      analogWrite(3,0);
    } else
    
    if (state == NAVIGATION) {
      int dir = irsensor();
      switch (dir) {
        case BACKWARD:
          digitalWrite(22,LOW);            // HIGH for forward LOW for reverse
          digitalWrite(23,LOW);            // HIGH for forward LOW for reverse
          analogWrite(2,POWER/2);
          analogWrite(3,POWER/2);
          timer_set(&t_movement, TIME_BACK*CLOCK_SECOND);
          #if DEBUG_THREADS
            report(OPS_DEBUG, "drive -- backward");
          #endif
          break;
        case STOP:  // Currently only called in testing
          analogWrite(2,0);
          analogWrite(3,0);
          timer_set(&t_movement, TIME_STOP*CLOCK_SECOND);
          #if DEBUG_THREADS
            report(OPS_DEBUG, "drive -- stop");
          #endif
          break;
        case LEFT:
          digitalWrite(22,HIGH);
          digitalWrite(23,LOW);
          analogWrite(2,TURNPOWER/2);
          analogWrite(3,TURNPOWER/2);
          timer_set(&t_movement, TIME_TURN*CLOCK_SECOND);
          timer_set(&t_no_adjust, TIME_MOVE*CLOCK_SECOND);
          #if DEBUG_THREADS
            report(OPS_DEBUG, "drive -- left");
          #endif
          break;
        case RIGHT:
          digitalWrite(22,LOW);
          digitalWrite(23,HIGH);
          analogWrite(2,TURNPOWER/2);
          analogWrite(3,TURNPOWER/2);
          timer_set(&t_movement, TIME_TURN*CLOCK_SECOND);
          timer_set(&t_no_adjust, TIME_MOVE*CLOCK_SECOND);
          #if DEBUG_THREADS
            report(OPS_DEBUG, "drive -- right");
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
              report(OPS_DEBUG, "drive -- forward");
            #endif
          }
          // no timer so IR sensors are constantly checked when moving forward
      }
    } else // endif state==NAVIGATION
    
    if (state == MANUAL) {
      timer_set(&t_no_adjust, 0);  // expire this timer for now

      int l = (int)(manual_l*POWER);
      int r = (int)(manual_r*POWER);
      digitalWrite(22, l >= 0 ? HIGH : LOW);
      digitalWrite(23, r >= 0 ? HIGH : LOW);
      analogWrite(2, abs(l));
      analogWrite(3, abs(r));

      #if DEBUG_ARGS
        Serial.print(OPS_DEBUG); Serial.print("Drive set: "); Serial.print(l); Serial.print(", "); Serial.println(r);
      #endif

      // reset, stop next loop until there is a new command
      manual_l = 0.0;
      manual_r = 0.0;
      manual_d = 0.0;
      state = WAITING;
      
      timer_set(&t_movement, manual_d*CLOCK_SECOND);
    } else // endif state==MANUAL

    if (state == FOLLOW) {
      // TODO: Followme code here
    } // endif state==FOLLOW
    
    timer_reset(&t_main);
  }
  PT_END(pt);
}

static PT_THREAD(sensorThread(struct pt *pt)){
  static struct timer t;
  PT_BEGIN(pt);

  timer_set(&t, SENSOR_RATE*CLOCK_SECOND);
  
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
      report(OPS_DEBUG, "sensors updated");
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
      Serial.print(OPS_DEBUG);
      Serial.print("Fix: "); Serial.print((int)GPS.fix);
      Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
      Serial.print(OPS_DEBUG);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      if (GPS.fix) {
        Serial.print(OPS_DEBUG);
        Serial.print("Location ");
        Serial.print(GPS.latitudeDegrees, 4);
        Serial.print(", "); 
        Serial.println(GPS.longitudeDegrees, 4);
      } else {
        Serial.print(OPS_DEBUG);
        Serial.println("no fix yet");
      }
    #endif

    #if USE_IMU
      Serial.print(OPS_DEBUG);
      Serial.print("Heading: ");
      Serial.println(orientation.heading);
    #endif
    
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
  if ( distance0 > IR_THRESH_NEAR || distance1 > IR_THRESH_NEAR  ){ 
    if ( lastsensorpindata0 > IR_THRESH_NEAR || lastsensorpindata1 > IR_THRESH_NEAR ){
      setPreviousPins( 0, 0, 0, 0);
      return BACKWARD;
    }
    /*else {
      setPreviousPins( distance0, distance1, distance2, distance3);
      return STOP;
    }*/
  } 

  // obstacle in front
  else if ( distance0 > IR_THRESH_FAR || distance1 > IR_THRESH_FAR ) {
    if ( lastsensorpindata0 > IR_THRESH_FAR || lastsensorpindata1 > IR_THRESH_FAR ){
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
  else if ( distance2 > IR_THRESH_NEAR || distance3 > IR_THRESH_NEAR ) {
    if ( lastsensorpindata2 > IR_THRESH_NEAR || lastsensorpindata3 > IR_THRESH_NEAR ){
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

float rad2deg(float theta){
  return theta*180.0/M_PI;
}

float deg2rad(float theta){
  return theta*M_PI/180.0;
}

float normalize_angle(float theta){
  // normalize angle from 0 to 360
  return theta < 0 ? theta+360.0 : theta;
}

float getBearingTo(float det_lat, float det_lon){
  // bearing is flipped to counterclockwise to match heading
  return -calcBearing(currLat, currLon, det_lat, det_lon);
}

float calcBearing(float start_lat, float start_lon, float det_lat, float det_lon){
  //  get bearing based on two GPS locations - DEGREES
  start_lat = deg2rad(start_lat);
  start_lon = deg2rad(start_lon);
  det_lat = deg2rad(det_lat);
  det_lon = deg2rad(det_lon);
  float y = sin(det_lon - start_lon) * cos(det_lat);
  float x = cos(start_lat)*sin(det_lat) - sin(start_lat)*cos(det_lat)*cos(det_lon - start_lon);
  float theta = atan2(y, x);
  return normalize_angle(rad2deg(theta));
}

int adjustGoal(){
  int dir = -1;

  if(abs(orientation.heading - goalBearing) > HEADING_PRECISION){
    float diff = goalBearing - orientation.heading;
    if(diff < 0)
      diff += 360;
    if(diff > 180)
      dir = LEFT;
    else
      dir = RIGHT;
    
    if(dir == LEFT){
      // LEFT
      digitalWrite(22,HIGH);
      digitalWrite(23,LOW);
      analogWrite(2,TURNPOWER/2);
      analogWrite(3,TURNPOWER/2);
      #if DEBUG_THREADS
        report(OPS_DEBUG, "adjust -- left");
      #endif
    }
    else{
      // RIGHT
      digitalWrite(22,LOW);
      digitalWrite(23,HIGH);
      analogWrite(2,TURNPOWER/2);
      analogWrite(3,TURNPOWER/2);
      #if DEBUG_THREADS
        report(OPS_DEBUG, "adjust -- right");
      #endif
    } 
  } else {
    // Done Adjusting, move FORWARD
    digitalWrite(22,HIGH);
    digitalWrite(23,HIGH);
    analogWrite(2,POWER);
    analogWrite(3,POWER);
    #if DEBUG_THREADS
      report(OPS_DEBUG, "to goal -- forward");
    #endif
  }
}
