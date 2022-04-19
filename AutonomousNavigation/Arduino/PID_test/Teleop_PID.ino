#include <ros.h>

#include <PID_v1.h>
#include <Wire.h>
#include <DualVNH5019MotorShieldMod3.h>

#include <Adafruit_BNO055.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();

/*
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
*/

double yaw_input = 0, yaw_output = 0, yaw_setpoint = 0;
double yaw_kP = 5.5, yaw_kI = 1, yaw_kD = 0;
                  
PID yaw_PID (&yaw_input, &yaw_output, &yaw_setpoint, 
              yaw_kP, yaw_kI, yaw_kD, DIRECT);

void setup() {
  Serial.begin(115200);
  //md.init();

  InitIMU();
  InitPID();
  delay(1000);
}

void InitIMU() {
  Wire.beginTransmission(0x28);
  if (!bno.begin()) {
    Serial.print("No BNO055 detected. Check your wiring or I2C address (0x28)");
    while (1);
  }
  bno.setExtCrystalUse(true);
}


void InitPID() {
  yaw_PID.SetMode(AUTOMATIC);
  yaw_PID.SetOutputLimits(-100, 100);
}



void loop() {
  IMU_Angle();
  yaw_PID.Compute();
  Output(yaw_output);
  delay(100);
}


void IMU_Angle() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  yaw_input = euler.x();
  Serial.print("Angle: "); Serial.println(yaw_input);
  yaw_setpoint = 30-5;
}

void Output(int output) {
  if (output > 0) {
    Serial.print("Speed: "); Serial.println(output);
    /*
    md.setM1Speed(output); //BL
    md.setM2Speed(output); //FL
    md.setM3Speed(output); //FR
    md.setM4Speed(output); //BR
    */
  }
}
