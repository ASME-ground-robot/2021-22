#include <ros.h>
#include <rover/sensors2smach.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>

ros::NodeHandle nh;
rover::sensors2smach imu_angle;
ros::Publisher anglepub("sensors2smach", &imu_angle);

/* IMU Connections
   ===========
   Connect SCL to analog 5; SCL 21 on mega
   Connect SDA to analog 4; SDA 20 on mega
   Connect Vin to 5V DC
   Connect GND to common ground
*/
#define BNO055_SAMPLERATE_DELAY_MS (200)
Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() {
  Serial.begin(115200);
  InitializeIMU();
  delay(1000);
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(anglepub);
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

  anglepub.publish(&imu_angle);
  nh.spinOnce();
}

void IMU_Data() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float new_angle = euler.x() + 0.08;
  if (new_angle > 360) {
    new_angle = new_angle - 360;
  }
  imu_angle.current_yaw = new_angle;
  Serial.print(" IMU Angle: ");
  Serial.print(new_angle);
  Serial.println("\t");
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
