#include <NMEAGPS.h>

/* GPS Connections
   ===========
   Connect TX to pin 8; 19(RX1) for Mega
   Connect RX to pin 7; 18(TX1) for Mega
   Connect Vin to 5V DC
   Connect GND to common ground
*/

NMEAGPS gps;
#define gpsPort Serial1

void setup() {
  Serial.begin(115200);
  gpsPort.begin(9600);
  delay(1000);
}


void loop() {
  if (gps.available(gpsPort)) {
    gps_fix fix = gps.read();
    if (fix.valid.location) {
      Serial.print("Latitude: "); Serial.println( fix.latitude(), 6 );
      Serial.print("Longitude: "); Serial.println( fix.longitude(), 6 );
    }
  }
}
