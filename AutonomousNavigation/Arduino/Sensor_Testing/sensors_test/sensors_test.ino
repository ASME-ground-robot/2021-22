#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>

/* IMU Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect Vin to 5V DC
   Connect GND to common ground
*/
#define BNO055_SAMPLERATE_DELAY_MS (200)
Adafruit_BNO055 bno = Adafruit_BNO055();

/* GPS Connections
   ===========
   Connect TX to pin 8
   Connect RX to pin 7
   Connect Vin to 5V DC
   Connect GND to common ground
*/
SoftwareSerial gps(8, 7); // (TX, RX);
Adafruit_GPS GPS(&gps);
#define GPSECHO  false  // raw data on or off; for troubleshooting

uint32_t timer = millis();

void setup() {
  Serial.begin(115200);
  InitializeIMU();
  InitializeGPS();
  delay(1000);
}

void InitializeIMU() {
  Wire.beginTransmission(0x28);
  if (!bno.begin()) {
    Serial.print("Oof, no BNO055 detected ... Check your wiring or I2C ADDR, dumbass! You're garbage. Git gud, kid. ");
    while (1);
  }
  delay (1000);
  /* 
// Display the current temperature
  int8_t temp = bno.getTemp();
  Serial.print("IMU Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");
  */
  bno.setExtCrystalUse(true);
}

void InitializeGPS() {
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //GPS minimum raw data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); //1 Hz sample rate
  GPS.sendCommand(PGCMD_ANTENNA); // antenna status
  delay(1000);
}


void loop() {
  IMU_Data();
  GPS_Data();
}

void IMU_Data() {
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
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void GPS_Data() {
  char c = GPS.read();
  if ((c) && (GPSECHO)) {
    Serial.write(c);
  }
  if (GPS.newNMEAreceived()) {
    //Serial.println(GPS.lastNMEA()); // sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // fails to parse a sentence, waits for another
      return;
  }

  // every n seconds, print out the current stats
  if (millis() - timer > 1000) {
    timer = millis(); // reset the timer
/*
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
*/
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Latitude: ");
      Serial.print(GPS.latitude, 4); Serial.println(GPS.lat);
      Serial.print("Longitude: ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
/*
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
*/
    }
  }
}
