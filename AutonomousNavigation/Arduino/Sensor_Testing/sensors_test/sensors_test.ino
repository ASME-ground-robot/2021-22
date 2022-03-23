let#include <NMEAGPS.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>

/* IMU Connections
   ===========
   Connect SCL to analog 5; 21 on Mega
   Connect SDA to analog 4; 20 on Mega
   Connect Vin to 5V DC
   Connect GND to common ground
*/
#define BNO055_SAMPLERATE_DELAY_MS (500)
Adafruit_BNO055 bno = Adafruit_BNO055();

/* GPS Connections
   ===========
   Connect TX to pin 8; 19(RX1) on Mega
   Connect RX to pin 7; 18(TX1) on Mega
   Connect Vin to 5V DC
   Connect GND to common ground
*/

NMEAGPS gps;
#define gpsPort Serial1
//SoftwareSerial gps(8, 7); // (TX, RX); Uno only
//Adafruit_GPS GPS(&gps)
//#define GPSECHO false  // raw data on or off; for troubleshooting
uint32_t timer = millis();

void setup() {
  Serial.begin(115200);
  InitializeSensors();
}

void InitializeSensors() {
  Wire.beginTransmission(0x28);
  if (!bno.begin()) {
    Serial.print("Oof, no BNO055 detected ... Check your wiring or I2C ADDR, dumbass! You're garbage. Git gud, kid. ");
    while (1);
  }
  bno.setExtCrystalUse(true);
  gpsPort.begin(9600);
/*
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //GPS minimum raw data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); //1 Hz sample rate
  GPS.sendCommand(PGCMD_ANTENNA); // antenna status
*/
}

void loop() {
  Sensor_Data();
}

void Sensor_Data() {
  if (gps.available(gpsPort)) {
    gps_fix fix = gps.read();
    if (fix.valid.location) {
      Serial.print("Latitude: "); Serial.println( fix.latitude(), 6 );
      Serial.print("Longitude: "); Serial.println( fix.longitude(), 6 );
    }
    //  Take an IMU sample too.
      // Possible vector values can be:
      // - VECTOR_ACCELEROMETER - m/s^2
      // - VECTOR_MAGNETOMETER  - uT
      // - VECTOR_GYROSCOPE     - rad/s
      // - VECTOR_EULER         - degrees
      // - VECTOR_LINEARACCEL   - m/s^2
      // - VECTOR_GRAVITY       - m/s^2

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.print(" IMU Angle: ");
    Serial.println(euler.x());
    Serial.println("");
  }
/*
  char c = GPS.read();
  if ((c) && (GPSECHO)) {
    Serial.print(c);
  }
  if (GPS.newNMEAreceived()) {
    //Serial.println(GPS.lastNMEA()); // sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // fails to parse a sentence, waits for another
      return;
  }

  // every n seconds, print out the current stats
  if (millis() - timer > 500) {
    timer = millis(); // reset the timer
    
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Latitude: ");
      Serial.print(GPS.latitude, 4); Serial.println(GPS.lat);
      Serial.print("Longitude: ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      
      // Possible vector values can be:
      // - VECTOR_ACCELEROMETER - m/s^2
      // - VECTOR_MAGNETOMETER  - uT
      // - VECTOR_GYROSCOPE     - rad/s
      // - VECTOR_EULER         - degrees
      // - VECTOR_LINEARACCEL   - m/s^2
      // - VECTOR_GRAVITY       - m/s^2

      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      Serial.print(" IMU Angle: ");
      Serial.print(euler.x());
      Serial.println("\t");
    }
  }
*/
}
