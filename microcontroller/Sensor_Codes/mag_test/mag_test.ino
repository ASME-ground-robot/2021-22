/* BNO055 Connections
   ===========
   Connect SCL to analog 5; SCL 21 on mega
   Connect SDA to analog 4; SDA 20 on mega
   Connect Vin to 5V DC
   Connect GND to common ground
*/
#include <Wire.h>
//#include <utility/imumaths.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();

#include <ros.h>
#include <rover/sensors2smach.h>
ros::NodeHandle nh;
rover::sensors2smach sensor_data;
ros::Publisher sensorpub("sensors2smach", &sensor_data);

void setup() {
  Serial.begin(115200);
  InitializeIMU();
  delay(1000);
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(sensorpub);
}

void InitializeIMU() {
  Wire.beginTransmission(0x28);
  if (!bno.begin()) {
    Serial.print("Oof, no BNO055 detected ... Check your wiring or I2C ADDR, dumbass! You're garbage. Git gud, kid. ");
    while (1);
  }
  bno.setExtCrystalUse(true);
}

void loop() {
  IMU_Data();

  sensorpub.publish(&sensor_data);
  nh.spinOnce();
}

void IMU_Data() {
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  /*
  Serial.print(" X: ");
  Serial.print(euler.x());
  Serial.print("  Y: ");
  Serial.println(euler.y());
  */
  delay(BNO055_SAMPLERATE_DELAY_MS);

  float headingDeg = atan2(euler.y(), euler.x())*180/PI;
  float declinationAngle = 11.66;
  
  headingDeg = headingDeg + declinationAngle;
  
  if (headingDeg < 0) {
    headingDeg = headingDeg + 360;
  }
  if (headingDeg > 360) {
    headingDeg = headingDeg - 360;
  }
  sensor_data.current_yaw = headingDeg;
  sensorpub.publish(&sensor_data);
  Serial.print("Angle: "); Serial.println(headingDeg);
}
