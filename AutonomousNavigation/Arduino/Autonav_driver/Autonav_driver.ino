#include <DualVNH5019MotorShieldMod3.h>

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <rover/sensors2smach.h>
#include <rover/smach2controls.h>

#include <NMEAGPS.h>
#include <Wire.h>
#include <LIS3MDL.h>

ros::NodeHandle nh;
rover::sensors2smach sensor_data;
ros::Publisher sensorpub("sensors2smach", &sensor_data);

// Green: Back Left Connections
int const INA1 = 31;
int const INB1 = 33;
int const EN1DIAG1 = 29;
int const CS1 = A0;

//Yellow: Front Left Connections
int const INA2 = 25;
int const INB2 = 27;
int const EN2DIAG2 = 23;
int const CS2 = A1;

//Blue: Front Right Connections
int const INA3 = 30;
int const INB3 = 32;
int const EN3DIAG3 = 28;
int const CS3 = A2;

//Orange: Back Right Connections
int const INA4 = 24;
int const INB4 = 26;
int const EN4DIAG4 = 22;
int const CS4 = A3;

int const PWM1 = 12; //BL
int const PWM2 = 3; //FL
int const PWM3 = 9; //FR
int const PWM4 = 5; //BR

DualVNH5019MotorShieldMod3 md(INA1, INB1, EN1DIAG1, CS1, INA2, INB2, EN2DIAG2, CS2, INA3, INB3, EN3DIAG3, CS3, INA4, INB4, EN4DIAG4, CS4, PWM1, PWM2, PWM3, PWM4);

const int LED1 = 11; //Autonav Mode LED-> ON
int i = 0;
double Distance;
double Heading;
int count = 0;
int encoder_dist = 0;

/* Magnetometer Connections
   ===========
   Connect SCL to analog 5; 21 on Mega
   Connect SDA to analog 4; 20 on Mega
   Connect Vin to 5V DC
   Connect GND to common ground
*/
LIS3MDL mag;

/* GPS Connections
   ===========
   Connect TX to pin 8; 19(RX1) on Mega
   Connect RX to pin 7; 18(TX1) on Mega
   Connect Vin to 5V DC
   Connect GND to common ground
*/

NMEAGPS gps;
#define gpsPort Serial1

void Goals(const rover::smach2controls& setpoint_data) {
  Distance = setpoint_data.distance_setpoint;
  Heading = setpoint_data.yaw_setpoint;
}

ros::Subscriber <rover::smach2controls> sub("smach2controls", Goals);


void setup() {
  Serial.begin(115200);
  InitializeSensors();
  
  md.init();
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
  Sensor_Data();
  nh.spinOnce();
}

void Sensor_Data() {
  if (gps.available(gpsPort)) {
    gps_fix fix = gps.read();
    if (fix.valid.location) {
      mag.read();
      float headingRad = atan2(mag.m.y, mag.m.x);
      float headingDeg = headingRad*180/PI;
      float declinationAngle = 11.76666666666;
  
      headingDeg += declinationAngle;
  
      if (headingDeg < 0) {
        headingDeg += 360;
        }

      sensor_data.current_yaw = headingDeg;
      sensorpub.publish(&sensor_data);
      sensor_data.current_latitude = fix.latitude();
      sensorpub.publish(&sensor_data);
      sensor_data.current_longitude = fix.longitude();
      sensorpub.publish(&sensor_data);
      
      switch (count) {
        case 0: //turning right
        if (headingDeg < Heading) {
          md.setM1Speed(255); //BL
          md.setM2Speed(255); //FL
          md.setM3Speed(255); //FR
          md.setM4Speed(255); //BR
        }
        count++;
        break;

        case 1: //turning left
        if (headingDeg > Heading) {
          md.setM1Speed(-255); //BL
          md.setM2Speed(-255); //FL
          md.setM3Speed(-255); //FR
          md.setM4Speed(-255); //BR
        }
        //count++;
        break;
        
        case 2: //Drive to goal
        if (encoder_dist < Distance) {
          md.setM1Speed(100); //BL
          md.setM2Speed(100); //FL
          md.setM3Speed(100); //FR
          md.setM4Speed(100); //BR
        }
        break;
      }
    }
  }
}
