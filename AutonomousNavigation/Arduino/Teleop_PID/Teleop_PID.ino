#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <PID_v1.h>
//#include <Wire.h>
#include <DualVNH5019MotorShieldMod3.h>
//#include <Adafruit_BNO055.h>
//#define BNO055_SAMPLERATE_DELAY_MS (100)
//Adafruit_BNO055 bno = Adafruit_BNO055();

ros::NodeHandle nh;

// Green: Back Left
int const INA1 = 31; //BL
int const INB1 = 33; //BL
int const EN1DIAG1 = 29;
int const CS1 = A0;

//Yellow: Front Left 
int const INA2 = 25; //FL
int const INB2 = 27; //FL
int const EN2DIAG2 = 23;
int const CS2 = A1;

//Blue: Front Right
int const INA3 = 30; //FR
int const INB3 = 32; //FR
int const EN3DIAG3 = 28;
int const CS3 = A2;

//Orange: Back Right
int const INA4 = 24; //BR
int const INB4 = 26; //BR
int const EN4DIAG4 = 22;
int const CS4 = A3;

int const PWM1 = 12; //BL
int const PWM2 = 3; //FL
int const PWM3 = 9; //FR
int const PWM4 = 5; //BR

DualVNH5019MotorShieldMod3 md(INA1, INB1, EN1DIAG1, CS1, INA2, INB2, EN2DIAG2, CS2, INA3, INB3, EN3DIAG3, CS3, INA4, INB4, EN4DIAG4, CS4, PWM1, PWM2, PWM3, PWM4);
const int LED1 = 11;

double distance_input, distance_output, distance_setpoint;
double distance_kP = 5, distance_kI = 0, distance_kD = 0;

double yaw_input, yaw_output, yaw_setpoint;
double yaw_kP = 6, yaw_kI = 0.005, yaw_kD = 0;

PID distance_PID (&distance_input, &distance_output, &distance_setpoint, distance_kP, distance_kI, distance_kD, DIRECT);
PID yaw_PID (&yaw_input, &yaw_output, &yaw_setpoint, yaw_kP, yaw_kI, yaw_kD, DIRECT);

void Controller(const std_msgs::Int16MultiArray &msg) {
  int foward = msg.data[0];
  int reverse = msg.data[1];
  int left = msg.data[2];
  int right = msg.data[3];
  
  if (foward == 1) {
    distance_setpoint = distance_setpoint + 1;
  }
  else
  if (reverse == 1) {
    distance_setpoint = distance_setpoint - 1;
  }
  if (left == 1) {
    yaw_setpoint = 20;
  }
  if (right == 1) {
    yaw_setpoint = -20;
  }
  if (foward == 0 && reverse == 0 && left == 0 && right == 0) {
    md.setM1Speed(0);
    md.setM2Speed(0);
    md.setM3Speed(0);
    md.setM4Speed(0);
  }
  if (foward == 1 || reverse == 1 || left == 1 || right == 1) {
    digitalWrite(LED1,HIGH);
  }
  else {
    digitalWrite(LED1, LOW);
  }
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("py_control", &Controller);

void setup() {
  Serial.begin(115200);
  md.init();
  distance_PID.SetMode(AUTOMATIC);
  yaw_PID.SetMode(AUTOMATIC);
  distance_PID.SetOutputLimits(-200.00, 200.00);
  yaw_PID.SetOutputLimits(-200.00, 200.00);
  
  //InitializeIMU();
  distance_setpoint = 0;
  yaw_setpoint = 0;

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  delay(500);
}

/*
void InitializeIMU() {
  Wire.beginTransmission(0x28);
  if (!bno.begin()) {
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR");
    while (1);
  }
  bno.setExtCrystalUse(true);
}
*/

void loop() {
  //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  //yaw_input = euler.x();
  yaw_input = 0;
  distance_input = 0;

  Distance_Outputs();
  Yaw_Outputs();
  nh.spinOnce();
}


void Distance_Outputs() {
  //Controller();
  
  distance_PID.Compute();
  int distance_speed_L = distance_output;// + yaw_angle_output;
  int distance_speed_R = distance_output;// - yaw_angle_output;

  if (distance_speed_L > 200) {
    distance_speed_L = 200;
  }
  else if (distance_speed_L < -200) {
    distance_speed_L = -200;
  }
  if (distance_speed_R > 200) {
    distance_speed_R = 200;
  }
  else if (distance_speed_R < -200) {
    distance_speed_R = -200;
  }

  md.setM1Speed(distance_speed_L); //BL
  md.setM2Speed(distance_speed_L); //FL
  md.setM3Speed((-1)*distance_speed_R); //FR
  md.setM4Speed((-1)*distance_speed_R); //BR
}

void Yaw_Outputs() {
  //Controller();
  Path_Optimization();
  
  yaw_PID.Compute();
  int yaw_speed_L = yaw_output;// + yaw_angle_output;
  int yaw_speed_R = yaw_output;// - yaw_angle_output;

  if (yaw_speed_L > 200) {
    yaw_speed_L = 200;
  }
  else if (yaw_speed_L < -200) {
    yaw_speed_L = -200;
  }
  if (yaw_speed_R > 200) {
    yaw_speed_R = 200;
  }
  else if (yaw_speed_R < -200) {
    yaw_speed_R = -200;
  }

  md.setM1Speed(yaw_speed_L); //BL
  md.setM2Speed(yaw_speed_L); //FL
  md.setM3Speed(yaw_speed_R); //FR
  md.setM4Speed(yaw_speed_R); //BR
}

void Path_Optimization() {
  double half_circle_pos = 180.00;
  double half_circle_neg = -180.00;
  double yaw_error = yaw_setpoint - yaw_input;

  if (yaw_error > half_circle_pos) {
    yaw_error = yaw_error - 360;
  }
  else if (yaw_error < half_circle_neg) {
    yaw_error = yaw_error + 360;
  }
  yaw_setpoint = yaw_error + yaw_input;
}
