#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;


////////// Motor Driver Parameters //////////
#include <CustomDualG2HighPowerMotorShield.h>
DualG2HighPowerMotorShield18v22 md;

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


////////// Encoder Parameters //////////
#define ENC_RIGHT A15
#define ENC_LEFT A14

// publishes tick count to ROS.
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

// Counts ticks and determines the current tick value based on last value.
int currentstate_R;
int laststate_R;
int currentstate_L;
int laststate_L;

// Increment the number of ticks
void right_wheel_tick() {
  currentstate_R = analogRead(ENC_RIGHT);
  if (currentstate_R < laststate_R + 2) {
    right_wheel_tick_count.data++;
  }
  if (currentstate_R > laststate_R - 2) {
    right_wheel_tick_count.data--;
  }
  laststate_R = currentstate_R;
}
 
// Increment the number of ticks
void left_wheel_tick() {
  currentstate_L = analogRead(ENC_LEFT);
  if (currentstate_L < laststate_L + 2) {
    left_wheel_tick_count.data++;
  }
  if (currentstate_L > laststate_L - 2) {
    left_wheel_tick_count.data--;
  }
  laststate_L = currentstate_L;
}


////////// Wheel and Speed Parameters //////////

// wheel characteristics: radius and distance between wheels
double wheel_rad = 0.1397, wheel_dist = 0;

//min and max PWM speed values from motor drivers
int max_vel = 400;
int min_vel = -400;

// speed values
double w_r = 0, w_l = 0;
double vel_yaw = 0, vel_dist = 0;


////////// Velocity Data From Twist //////////
void Twist( const geometry_msgs::Twist& msg) {
  vel_yaw = msg.angular.z;
  vel_dist = msg.linear.x;
  w_r = (vel_dist/wheel_rad) + ((vel_yaw*wheel_dist)/(2.0*wheel_rad));
  w_l = (vel_dist/wheel_rad) - ((vel_yaw*wheel_dist)/(2.0*wheel_rad));
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &Twist );


////////// Setup //////////
void setup() {
  Serial.begin(115200);
  md.init();
  // Uncomment to flip a motor's direction:
  md.flipM3(true);
  md.flipM4(true);
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  
}


////////// Loop //////////
void loop() {
  md.enableDrivers();
  delay(1);

  int vel_l = w_l*10;
  int vel_r = w_r*10;
  
  // checks PWM is between -200 and 200.
  if (vel_l > 200 && vel_r > 200) {
    vel_l = 200;
    vel_r = 200;
  }
  if (vel_l < -200 && vel_r < -200) {
    vel_l = -200;
    vel_r = -200;
  }

  // sets velocity to PWM values
  md.setM1Speed(vel_l); //BL
  md.setM2Speed(vel_l); //FL
  md.setM3Speed(vel_r); //FR
  md.setM4Speed(vel_r); //BR
  stopIfFault();

  nh.spinOnce();
}
