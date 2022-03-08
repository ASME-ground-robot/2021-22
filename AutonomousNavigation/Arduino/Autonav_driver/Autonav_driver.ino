/* ASME 2019-2020 Controls Code
  
   Current Revision 02 12 21
      Added Cytron Motor Driver Library
      Pin Numbers for Arduino Mega
      Manual Input: w, a, s, d

   Revision 02 08 20
      Using Pre-Made PID Library
*/

#include <PID_v1.h>
#include <DualVNH5019MotorShieldMod3.h>
//#include <CytronMotorDriver.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();

int INA1 = 30; //FR
int INB1 = 32; //FR
int PWM1 = 9; //FR
int EN1DIAG1 = 28;

int INA2 = 24; //BR
int INB2 = 26; //BR
int PWM2 = 5; //BR
int EN2DIAG2 = 22;

int INA3 = 31; //FL
int INB3 = 33; //FL
int PWM3 = 6; //FL
int EN3DIAG3 = 29;

int INA4 = 25; //BL
int INB4 = 27; //BL
int PWM4 = 3; //BL
int EN4DIAG4 = 23;

DualVNH5019MotorShieldMod3 md(INA1, INB1, EN1DIAG1, INA2, INB2, EN2DIAG2, INA3, INB3, EN3DIAG3, INA4, INB4, EN4DIAG4, PWM1, PWM2, PWM3, PWM4);

//const int Lmotor_Pwm = 3;
//const int Lmotor_Digital = 8;
int final_Lmotor_speed = 0;
//CytronMD LMotor(PWM_DIR, Lmotor_Pwm, Lmotor_Digital);

//const int Rmotor_Pwm = 6;
//const int Rmotor_Digital = 24;
int final_Rmotor_speed = 0;
//CytronMD RMotor(PWM_DIR, Rmotor_Pwm, Rmotor_Digital);



double distance_input, distance_output, distance_setpoint;
double distance_kP = 5; //trial and error//
double distance_kI = 0; //trial and error//
double distance_kD = 0; //trial and error//

double yaw_angle_input, yaw_angle_output, yaw_angle_setpoint;
double yaw_angle_kP = 6;  //7*   //8
double yaw_angle_kI = 0.005;
double yaw_angle_kD = 0;  // 1*  realized ground acts as friction //1.1  

PID distance_PID     (&distance_input, &distance_output, &distance_setpoint,
                      distance_kP, distance_kI, distance_kD, DIRECT);
PID yaw_angle_PID    (&yaw_angle_input, &yaw_angle_output, &yaw_angle_setpoint,
                      yaw_angle_kP, yaw_angle_kI, yaw_angle_kD, DIRECT);



void setup() {
  Serial.begin(115200);
//  Serial.begin(9600);
  InitializeIMU();

//  pinMode(Lmotor_Pwm, OUTPUT);     //Are these needed?
//  pinMode(Lmotor_Digital, OUTPUT);
//  pinMode(Rmotor_Pwm, OUTPUT);
//  pinMode(Rmotor_Digital, OUTPUT);
  
  distance_PID.SetMode(AUTOMATIC);
  yaw_angle_PID.SetMode(AUTOMATIC);
  distance_PID.SetOutputLimits(-255.00, 255.00);
  yaw_angle_PID.SetOutputLimits(-125.00, 125.00);

  distance_setpoint = 0;
  yaw_angle_setpoint = 0;
  delay(3000);
}



//Turns IMU data collecting on
void InitializeIMU() {
  //Wire.begin();       //Can use this if dealing with only 1 sensor
  Wire.beginTransmission(0x28);
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Oof, no BNO055 detected ... Check your wiring or I2C ADDR, dumbass! You're garbage. Git gud, kid. ");
    while (1);
  }
  bno.setExtCrystalUse(true);
}



void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  yaw_angle_input = euler.x();
                            // this is where the //
  distance_input = 0;       //                   //
                            //encoder readings go//
  Outputs();
}



void Outputs() {
  Manual_Input();
  Path_Optimization();
  distance_PID.Compute();
  yaw_angle_PID.Compute();
  final_Lmotor_speed = distance_output;// + yaw_angle_output;
  final_Rmotor_speed = distance_output;// - yaw_angle_output;

  if (final_Lmotor_speed > 255) {
    final_Lmotor_speed = 255;
  }
  else if (final_Lmotor_speed < -255) {
    final_Lmotor_speed = -255;
  }
  if (final_Rmotor_speed > 255) {
    final_Rmotor_speed = 255;
  }
  else if (final_Rmotor_speed < -255) {
    final_Rmotor_speed = -255;
  }
  Serial.print("Left: ");
  Serial.print(final_Lmotor_speed);
  Serial.print("-------");
  Serial.print("Right: ");
  Serial.print(final_Rmotor_speed);
  Serial.print("-------");
  Serial.print(yaw_angle_input);
  Serial.println("-------");
  Serial.print(yaw_angle_setpoint);
  Serial.println("-------");

  md.setM1Speed((-1)*final_Lmotor_speed);
  md.setM2Speed((-1)*final_Lmotor_speed);
  md.setM3Speed((-1)*final_Rmotor_speed);
  md.setM4Speed((-1)*final_Rmotor_speed);
  
//  LMotor.setSpeed((-1)*final_Lmotor_speed);
//  RMotor.setSpeed((-1)*final_Rmotor_speed);
}



void Manual_Input() {
  int Manual_input = 0;
  if (Serial.available() > 0) {
    Manual_input = Serial.read();
    Serial.print("Manual_Input = ");
    Serial.println(Manual_input);
  }
  if (isEqual(Manual_input)) {
    switch (Manual_input) {
      case 119: //w       Forwards
        distance_setpoint = distance_setpoint + 15;
        break;

      case 97:  //a       Left 10 Degrees
        yaw_angle_setpoint = yaw_angle_setpoint - 30;
        break;

      case 115: //s       Backwards
        distance_setpoint = distance_setpoint - 15;
        break;

      case 100: //d       Right 10 Degrees
        yaw_angle_setpoint = yaw_angle_setpoint + 30;
        break;
    }
  }
}



//Checks if Manual_Input is one of the keys
bool isEqual(int x) {
  int Check[] = {119, 97, 115, 100};       //w,a,s,d

  for (int i = 0; i < 4; i++) {
    if (Check[i] = x) {
      return true;
    }
  }
  return false;
}


void Path_Optimization() {
  double half_circle_pos = 180.00;
  double half_circle_neg = -180.00;
  double yaw_angle_error = yaw_angle_setpoint - yaw_angle_input;

  if (yaw_angle_error > half_circle_pos) {
    yaw_angle_error = yaw_angle_error - 360;
  }
  else if (yaw_angle_error < half_circle_neg) {
    yaw_angle_error = yaw_angle_error + 360;
  }

  yaw_angle_setpoint = yaw_angle_error + yaw_angle_input;
}
