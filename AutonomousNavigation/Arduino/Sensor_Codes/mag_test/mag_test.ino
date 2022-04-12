#include <Wire.h>
#include <LIS3MDL.h>
LIS3MDL mag;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  if (!mag.init()) {
    Serial.println("Failed to detect and initialize magnetometer!");
    while (1);
  }
  mag.enableDefault();
}

void loop() {
  mag.read();
  float headingRad = atan2(mag.m.y, mag.m.x);
  float headingDeg = headingRad*180/PI;
  float declinationAngle = 11.76666666666;
  
  headingDeg += declinationAngle;
  
  if (headingDeg < 0) {
    headingDeg += 360;
  }

  //Serial.print("X: "); Serial.print(mag.m.x);
  //Serial.print("  Y: "); Serial.println(mag.m.y);
  //mag_angle = atan2(mag.m.x, mag.m.y)*180/pi;
  Serial.print("Angle: "); Serial.println(headingDeg);
  delay(100);
}
