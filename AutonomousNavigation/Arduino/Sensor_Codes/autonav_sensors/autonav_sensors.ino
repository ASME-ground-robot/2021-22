#include <ros.h>
#include <rover/sensors2smach.h>
#include <rover/smach2controls.h>

#include <NMEAGPS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();

ros::NodeHandle nh;
rover::sensors2smach sensor_data;
ros::Publisher sensorpub("sensors2smach", &sensor_data);

/* Magnetometer Connections
   ===========
   Connect SCL to analog 5; 21 on Mega
   Connect SDA to analog 4; 20 on Mega
   Connect Vin to 5V DC
   Connect GND to common ground
*/

/* GPS Connections
   ===========
   Connect TX to pin 8; 19(RX1) on Mega
   Connect RX to pin 7; 18(TX1) on Mega
   Connect Vin to 5V DC
   Connect GND to common ground
*/

NMEAGPS gps;
#define gpsPort Serial1

/*
double Distance;
double Heading;
void Outputs(const rover::smach2controls& setpoint_data) {
  Distance = setpoint_data.distance_setpoint;
  Heading = setpoint_data.yaw_setpoint;
  Serial.print("Distance Setpoint: "); Serial.println(Distance);
  Serial.print("Heading Setpoint: "); Serial.println(Heading);
}

ros::Subscriber <rover::smach2controls> sub("smach2controls", Outputs);
*/

void setup() {
  Serial.begin(115200);
  InitializeSensors();

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(sensorpub);
  //nh.subscribe(sub);
}

void InitializeSensors() {
  Wire.beginTransmission(0x28);
  if (!bno.begin()) {
    Serial.print("Oof, no BNO055 detected ... Check your wiring or I2C ADDR, dumbass! You're garbage. Git gud, kid. ");
    while (1);
  }
  bno.setExtCrystalUse(true);
  gpsPort.begin(9600);
}

void loop() {
  Sensor_Data();
  nh.spinOnce();
  delay(10);
}

void Sensor_Data() {
  if (gps.available(gpsPort)) {
    gps_fix fix = gps.read();
    if (fix.valid.location) {
      Serial.print("Latitude: "); Serial.println( fix.latitude(), 6 );
      Serial.print("Longitude: "); Serial.println( fix.longitude(), 6 );

      
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
      float headingDeg = atan2(euler.y(), euler.x())*180/PI;
      float declinationAngle = 11.76666666666;
  
      headingDeg = headingDeg + declinationAngle;
  
      if (headingDeg < 0) {
        headingDeg = headingDeg + 360;
      }
      if (headingDeg > 360) {
        headingDeg = headingDeg - 360;
      }

      Serial.print("Angle: "); Serial.println(headingDeg);
      Serial.println("");
      
      
      sensor_data.current_yaw = headingDeg;
      sensorpub.publish(&sensor_data);
      sensor_data.current_latitude = fix.latitude();
      sensorpub.publish(&sensor_data);
      sensor_data.current_longitude = fix.longitude();
      sensorpub.publish(&sensor_data);
     }
   }
 }
