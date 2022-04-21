#include <Wire.h>
#include <LIS3MDL.h>
#include <ros.h>
#include <rover/sensors2smach.h>

ros::NodeHandle nh;
rover::sensors2smach sensor_data;
ros::Publisher sensorpub("sensors2smach", &sensor_data);

LIS3MDL mag;
LIS3MDL::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mag.init();
  mag.enableDefault();
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(sensorpub);
}

void loop() {
  Data();
  nh.spinOnce();
}

void Data() {
  mag.read();
  int min_x = -1647;
  int min_y = -1614;

  int max_x = 2580;
  int max_y = 2508;
  
  float hard_x = (max_x + min_x)/2;
  float hard_y = (max_y + min_y)/2;
  
  float delta = (hard_x + hard_y)/2;
  float soft_x = delta/hard_x;
  float soft_y = delta/hard_y;

  float new_x = (mag.m.x - hard_x)*soft_x;
  float new_y = (mag.m.y - hard_y)*soft_y;
  
  
  float headingDeg = atan2(new_y, new_x)*180/PI;
  float declinationAngle = 12;
  
  headingDeg = headingDeg + declinationAngle;
  
  if (headingDeg < 0) {
    headingDeg = headingDeg + 360;
  }
  if (headingDeg > 360) {
    headingDeg = headingDeg - 360;
  }
  sensor_data.current_yaw = headingDeg;
  sensorpub.publish(&sensor_data);
  
  //Serial.print("X: "); Serial.print(new_x);
  //Serial.print("  Y: "); Serial.println(new_y);
  
  Serial.print("Angle: "); Serial.println(headingDeg);
  delay(100);
}
