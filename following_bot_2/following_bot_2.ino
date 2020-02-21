#define BLYNK_USE_DIRECT_CONNECT


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <BlynkSimpleSerialBLE.h>
#include "./TinyGPS.h"                 
#include "./geo_tracker.h"


TinyGPS gps;
  

Servo lidServo;
followLid lidState = CLOSED;


bool enabled = false;




SoftwareSerial bluetoothSerial(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);
SoftwareSerial nss(GPS_TX_PIN, 255);   


Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

GeoLoc checkGPS() {
  Serial.println("Reading onboard GPS: ");
  bool newdata = false;
  unsigned long start = millis();
  while (millis() - start < GPS_UPDATE_INTERVAL) {
    if (feedgps())
      newdata = true;
  }
  if (newdata) {
    return gpsdump(gps);
  }

  GeoLoc followLoc;
  followLoc.lat = 0.0;
  followLoc.lon = 0.0;
  
  return followLoc;
}


GeoLoc gpsdump(TinyGPS &gps) {
  float flat, flon;
  unsigned long age;
  
  gps.f_get_position(&flat, &flon, &age);

  GeoLoc followLoc;
  followLoc.lat = flat;
  followLoc.lon = flon;

  Serial.print(followLoc.lat, 7); Serial.print(", "); Serial.println(followLoc.lon, 7);

  return followLoc;
}


bool feedgps() {
  while (nss.available()) {
    if (gps.encode(nss.read()))
      return true;
  }
  return false;
}

// Lid Hook
BLYNK_WRITE(V0) {
  switch (lidState) {
    case OPENED:
      setServo(SERVO_LID_CLOSE);
      lidState = CLOSED;
      break;
    case CLOSED:
      setServo(SERVO_LID_OPEN);
      lidState = OPENED;
      break;
  }
}


BLYNK_WRITE(V1) {
  enabled = !enabled;
  

  stop();
}


BLYNK_WRITE(V2) {
  GpsParam gps(param);
  
  Serial.println("Received remote GPS: ");
  
  // Print 7 decimal places for Lat
  Serial.print(gps.getLat(), 7); Serial.print(", "); Serial.println(gps.getLon(), 7);

  GeoLoc phoneLoc;
  phoneLoc.lat = gps.getLat();
  phoneLoc.lon = gps.getLon();

  driveTo(phoneLoc, GPS_STREAM_TIMEOUT);
}


BLYNK_WRITE(V3) {
  Serial.print("Received Text: ");
  Serial.println(param.asStr());

  String rawInput(param.asStr());
  int colonIndex;
  int commaIndex;
  
  do {
    commaIndex = rawInput.indexOf(',');
    colonIndex = rawInput.indexOf(':');
    
    if (commaIndex != -1) {
      String latStr = rawInput.substring(0, commaIndex);
      String lonStr = rawInput.substring(commaIndex+1);

      if (colonIndex != -1) {
         lonStr = rawInput.substring(commaIndex+1, colonIndex);
      }
    
      float lat = latStr.toFloat();
      float lon = lonStr.toFloat();
    
      if (lat != 0 && lon != 0) {
        GeoLoc waypoint;
        waypoint.lat = lat;
        waypoint.lon = lon;
    
        Serial.print("Waypoint found: "); Serial.print(lat); Serial.println(lon);
        driveTo(waypoint, GPS_WAYPOINT_TIMEOUT);
      }
    }
    
    rawInput = rawInput.substring(colonIndex + 1);
    
  } while (colonIndex != -1);
}

void displayCompassDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif

float geoBearing(struct GeoLoc &a, struct GeoLoc &b) {
  float y = sin(b.lon-a.lon) * cos(b.lat);
  float x = cos(a.lat)*sin(b.lat) - sin(a.lat)*cos(b.lat)*cos(b.lon-a.lon);
  return atan2(y, x) * RADTODEG;
}

float geoDistance(struct GeoLoc &a, struct GeoLoc &b) {
  const float R = 6371000; // km
  float p1 = a.lat * DEGTORAD;
  float p2 = b.lat * DEGTORAD;
  float dp = (b.lat-a.lat) * DEGTORAD;
  float dl = (b.lon-a.lon) * DEGTORAD;

  float x = sin(dp/2) * sin(dp/2) + cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2);
  float y = 2 * atan2(sqrt(x), sqrt(1-x));

  return R * y;
}

float geoHeading() {

  sensors_event_t event; 
  mag.getEvent(&event);


  float heading = atan2(event.magnetic.y, event.magnetic.x);


  heading -= DECLINATION_ANGLE;
  heading -= COMPASS_OFFSET;
  

  if(heading < 0)
    heading += 2*PI;
    

  if(heading > 2*PI)
    heading -= 2*PI;
   

  float headingDegrees = heading * 180/M_PI; 


  while (headingDegrees < -180) headingDegrees += 360;
  while (headingDegrees >  180) headingDegrees -= 360;

  return headingDegrees;
}

void setServo(int pos) {
  lidServo.attach(SERVO_PIN);
  lidServo.write(pos);
  delay(2000);
  lidServo.detach();
}

void setSpeedMotorA(int speed) {
  digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  digitalWrite(MOTOR_A_IN_2_PIN, HIGH);
  

  analogWrite(MOTOR_A_EN_PIN, speed + MOTOR_A_OFFSET);
}

void setSpeedMotorB(int speed) {
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, HIGH);
  

  analogWrite(MOTOR_B_EN_PIN, speed + MOTOR_B_OFFSET);
}

void setSpeed(int speed)
{
  setSpeedMotorA(speed);


  setSpeedMotorB(speed);
}

void stop() {
  // now turn off motors
  digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  digitalWrite(MOTOR_A_IN_2_PIN, LOW);  
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, LOW);
}

void drive(int distance, float turn) {
  int fullSpeed = 230;
  int stopSpeed = 0;

  // drive to location
  int s = fullSpeed;
  if ( distance < 8 ) {
    int wouldBeSpeed = s - stopSpeed;
    wouldBeSpeed *= distance / 8.0f;
    s = stopSpeed + wouldBeSpeed;
  }
  
  int autoThrottle = constrain(s, stopSpeed, fullSpeed);
  autoThrottle = 230;

  float t = turn;
  while (t < -180) t += 360;
  while (t >  180) t -= 360;
  
  Serial.print("turn: ");
  Serial.println(t);
  Serial.print("original: ");
  Serial.println(turn);
  
  float t_modifier = (180.0 - abs(t)) / 180.0;
  float autoSteerA = 1;
  float autoSteerB = 1;

  if (t < 0) {
    autoSteerB = t_modifier;
  } else if (t > 0){
    autoSteerA = t_modifier;
  }

  Serial.print("steerA: "); Serial.println(autoSteerA);
  Serial.print("steerB: "); Serial.println(autoSteerB);

  int speedA = (int) (((float) autoThrottle) * autoSteerA);
  int speedB = (int) (((float) autoThrottle) * autoSteerB);
  
  setSpeedMotorA(speedA);
  setSpeedMotorB(speedB);
}

void driveTo(struct GeoLoc &loc, int timeout) {
  nss.listen();
  GeoLoc followLoc = checkGPS();
  bluetoothSerial.listen();

  if (followLoc.lat != 0 && followLoc.lon != 0 && enabled) {
    float d = 0;

    do {
      nss.listen();
      followLoc = checkGPS();
      bluetoothSerial.listen();
      
      d = geoDistance(followLoc, loc);
      float t = geoBearing(followLoc, loc) - geoHeading();
      
      Serial.print("Distance: ");
      Serial.println(geoDistance(followLoc, loc));
    
      Serial.print("Bearing: ");
      Serial.println(geoBearing(followLoc, loc));

      Serial.print("heading: ");
      Serial.println(geoHeading());
      
      drive(d, t);
      timeout -= 1;
    } while (d > 3.0 && enabled && timeout>0);

    stop();
  }
}

void setupCompass() {

  if(!mag.begin())
  {

    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  

  displayCompassDetails();
}

void setup()
{

  setupCompass();


  pinMode(MOTOR_A_EN_PIN, OUTPUT);
  pinMode(MOTOR_B_EN_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_2_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_2_PIN, OUTPUT);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);


  Serial.begin(4800);

  //GPS
  nss.begin(9600);

  //Bluetooth
  bluetoothSerial.begin(9600);
  Blynk.begin(bluetoothSerial, auth);
}


void testDriveNorth() {
  float heading = geoHeading();
  int testDist = 10;
  Serial.println(heading);
  
  while(!(heading < 5 && heading > -5)) {
    drive(testDist, heading);
    heading = geoHeading();
    Serial.println(heading);
    delay(500);
  }
  
  stop();
}

void loop()
{
  Blynk.run();
}
