#include <CustomDualG2HighPowerMotorShield.h>
DualG2HighPowerMotorShield18v22 md;

#include <PID_v1.h>

#include <Wire.h>
#include <Adafruit_BNO055.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();

double yaw_input = 0, yaw_output = 0, yaw_setpoint = 0;
double yaw_kP = 5.5, yaw_kI = 1, yaw_kD = 0;
                  
PID yaw_PID (&yaw_input, &yaw_output, &yaw_setpoint, 
              yaw_kP, yaw_kI, yaw_kD, DIRECT);

void setup() {
  Serial.begin(115200);
  md.init();

  // Uncomment to flip a motor's direction:
  md.flipM3(true);
  md.flipM4(true);
  
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
  md.enableDrivers();
  delay(1);
    
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
    md.setM1Speed(output); //BL
    md.setM2Speed(output); //FL
    md.setM3Speed(output); //FR
    md.setM4Speed(output); //BR
  }
}
