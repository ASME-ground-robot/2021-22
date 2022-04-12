#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

/* GPS Connections
   ===========
   Connect TX to pin 8; 19(RX1) for Mega
   Connect RX to pin 7; 18(TX1) for Mega
   Connect Vin to 5V DC
   Connect GND to common ground
*/
//SoftwareSerial gps(8, 7); // (TX, RX);
//Adafruit_GPS GPS(&gps);
Adafruit_GPS GPS(&Serial1);
#define GPSECHO  false  // raw data on or off; for troubleshooting

uint32_t timer = millis();

void setup() {
  Serial.begin(115200);
  InitializeGPS();
  delay(1000);
}

void InitializeGPS() {
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //GPS minimum raw data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); //1 Hz sample rate
  GPS.sendCommand(PGCMD_ANTENNA); // antenna status
  delay(1000);
}


void loop() {
  GPS_Data();
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
    
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Latitude: ");
      Serial.print(GPS.latitude, 4); Serial.println(GPS.lat);
      Serial.print("Longitude: ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);

      Serial.println("");
    }
  }
}
