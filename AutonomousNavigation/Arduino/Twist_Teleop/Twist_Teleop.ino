#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

// library to control motors
#include <CustomDualG2HighPowerMotorShield.h>
DualG2HighPowerMotorShield18v22 md;

double w_r = 0, w_l = 0;
double vel_yaw = 0, vel_dist = 0;

// wheel characteristics: radius and distance between wheels
double wheel_rad = 0.1397, wheel_dist = 0; //m
//min and max PWM speed values from motor drivers
int max_vel = 400;
int min_vel = -400;

const int LED = 53;

// velocity data
void Twist( const geometry_msgs::Twist& msg) {
  vel_yaw = msg.angular.z;
  vel_dist = msg.linear.x;
  w_r = (vel_dist/wheel_rad) + ((vel_yaw*wheel_dist)/(2.0*wheel_rad));
  w_l = (vel_dist/wheel_rad) - ((vel_yaw*wheel_dist)/(2.0*wheel_rad));
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &Twist );

// checks if theres faults in motors to turn them off
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
  md.flipM3(true);
  md.flipM4(true);
  
  pinMode(LED, OUTPUT);
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  
}

void loop() {
  md.enableDrivers();
  delay(1);

  int vel_l = w_l*10;
  int vel_r = w_r*10;
  
  if (vel_l > 200 && vel_r > 200) {
    vel_l = 200;
    vel_r = 200;
  }
  if (vel_l < -200 && vel_r < -200) {
    vel_l = -200;
    vel_r = -200;
  }
  
  md.setM1Speed(vel_l); //BL
  md.setM2Speed(vel_l); //FL
  md.setM3Speed(vel_r); //FR
  md.setM4Speed(vel_r); //BR
  stopIfFault();

  digitalWrite(LED, HIGH);
  nh.spinOnce();
}
