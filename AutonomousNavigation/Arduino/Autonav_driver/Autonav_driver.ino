#include <CustomDualG2HighPowerMotorShield.h>
DualG2HighPowerMotorShield18v22 md;

#include <ros.h>
#include <rover/sensors2smach.h>
#include <rover/smach2controls.h>
ros::NodeHandle nh;
rover::sensors2smach sensor_data;
ros::Publisher sensorpub("sensors2smach", &sensor_data);

#include <NMEAGPS.h>
#include <Wire.h>
#include <LIS3MDL.h>

/* Magnetometer Connections
   ===========
   Connect SCL to analog 5; 21 on Mega
   Connect SDA to analog 4; 20 on Mega
   Connect Vin to 5V DC
   Connect GND to common ground
*/
LIS3MDL mag;
LIS3MDL::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};

/* GPS Connections
   ===========
   Connect TX to pin 8; 19(RX1) on Mega
   Connect RX to pin 7; 18(TX1) on Mega
   Connect Vin to 5V DC
   Connect GND to common ground
*/

NMEAGPS gps;
#define gpsPort Serial1

const int LED1 = 11; //Autonav Mode LED-> ON
int i = 0;
double Distance;
double Heading;
int count = 0;
int encoder_dist = 0;


void Goals(const rover::smach2controls& setpoint_data) {
  Distance = setpoint_data.distance_setpoint;
  Heading = setpoint_data.yaw_setpoint;
}

ros::Subscriber <rover::smach2controls> sub("smach2controls", Goals);


void stopIfFault() {
  if (md.getM1Fault()) {
    md.disableDrivers();
    delay(1);
    while (1);
  }
  if (md.getM2Fault()) {
    md.disableDrivers();
    delay(1);
    while (1);
  }
  if (md.getM3Fault()) {
    md.disableDrivers();
    delay(1);
    while (1);
  }
  if (md.getM4Fault()) {
    md.disableDrivers();
    delay(1);
    while (1);
  }
}


void setup() {
  Serial.begin(115200);
  md.init();
  // Uncomment to flip a motor's direction:
  //md.flipM3(true);
  //md.flipM4(true);
  
  InitializeSensors();
  
  pinMode(LED1, OUTPUT);
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(sensorpub);
  nh.subscribe(sub);
}

void InitializeSensors() {
  Wire.begin();
  if (!mag.init()) {
    Serial.println("Failed to detect and initialize magnetometer!");
    while (1);
  }
  mag.enableDefault();
  gpsPort.begin(9600);
}


void loop() {
  Outputs();
  nh.spinOnce();
}

void Outputs() {
  md.enableDrivers();
  delay(1);
  
  if (gps.available(gpsPort)) {
    gps_fix fix = gps.read();
    if (fix.valid.location) {
      
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
      sensor_data.current_latitude = fix.latitude();
      sensorpub.publish(&sensor_data);
      sensor_data.current_longitude = fix.longitude();
      sensorpub.publish(&sensor_data);
      
      if (count == 0) {
        if (headingDeg < Heading) {
          Serial.println("Turning right...");
          md.setM1Speed(50); //BL
          md.setM2Speed(50); //FL
          md.setM3Speed(50); //FR
          md.setM4Speed(50); //BR
          stopIfFault();
        }
        else if (headingDeg > Heading) {
          Serial.println("Turning left...");
          md.setM1Speed(-50); //BL
          md.setM2Speed(-50); //FL
          md.setM3Speed(-50); //FR
          md.setM4Speed(-50); //BR
          stopIfFault();
        }
        else {
          md.setM1Speed(0); //BL
          md.setM2Speed(0); //FL
          md.setM3Speed(0); //FR
          md.setM4Speed(0); //BR
          count = 1;
        }
      }
      if (count == 1) {
        if (encoder_dist < Distance) {
          Serial.println("Turning right...");
          md.setM1Speed(50); //BL
          md.setM2Speed(50); //FL
          md.setM3Speed(-50); //FR
          md.setM4Speed(-50); //BR
          stopIfFault();
        }
        else {
          md.setM1Speed(0); //BL
          md.setM2Speed(0); //FL
          md.setM3Speed(0); //FR
          md.setM4Speed(0); //BR
          count = 2;
        }
      }
    }
  }
}
