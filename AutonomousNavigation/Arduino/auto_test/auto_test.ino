#include <CustomDualG2HighPowerMotorShield.h>
DualG2HighPowerMotorShield18v22 md;

#include <ros.h>
#include <rover/smach2controls.h>
ros::NodeHandle nh;

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();

#include <PID_v1.h>
double input = 0, output = 0, setpoint = 0;
double kP = 5.5, kI = 1, kD = 0;

PID Dist_PID (&input, &output, &setpoint, 
              kP, kI, kD, DIRECT);

int i = 0;
int count = 0;
double goal_yaw;
double goal_distance;

void Goals(const rover::smach2controls& setpoint_data) {
  //goal_yaw = setpoint_data.yaw_setpoint;
  //goal_distance = setpoint_data.distance_setpoint;
}

ros::Subscriber <rover::smach2controls> sub("smach2controls", Goals);

#define ENCA A15
int tick = 0;
int currentstate;
int laststate;

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
  
  InitializeSensor();
  delay(1000);
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
}

void InitializeSensor() {
  Wire.beginTransmission(0x28);
  if (!bno.begin()) {
    Serial.print("Oof, no BNO055 detected ... Check your wiring or I2C ADDR, dumbass! You're garbage. Git gud, kid. ");
    while (1);
  }
  bno.setExtCrystalUse(true);

  pinMode(ENCA, INPUT);
  laststate = analogRead(ENCA); 
}


void loop() {
  Outputs();
  nh.spinOnce();
}

void Outputs() {
  md.enableDrivers();
  delay(1);
  goal_yaw = 5;
  goal_distance = 2;
  
  if (count == 0) {
    /*
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    float headingDeg = atan2(euler.y(), euler.x())*180/PI;
    float declinationAngle = 11.66;
  
    headingDeg = headingDeg + declinationAngle;
    if (headingDeg < 0) {
      headingDeg = headingDeg + 360;
    }
    if (headingDeg > 360) {
      headingDeg = headingDeg - 360;
    }
    */
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float headingDeg = euler.x() + 0.08;
    if (headingDeg > 360) {
      headingDeg = headingDeg - 360;
    }
    
    if (headingDeg < goal_yaw - 0.5) {
      Serial.println("Turning right...");
      md.setM1Speed(50); //BL
      md.setM2Speed(50); //FL
      md.setM3Speed(-50); //FR
      md.setM4Speed(-50); //BR
      stopIfFault();
    }
    else if (headingDeg > goal_yaw + 0.5) {
      Serial.println("Turning left...");
      md.setM1Speed(-50); //BL
      md.setM2Speed(-50); //FL
      md.setM3Speed(50); //FR
      md.setM4Speed(50); //BR
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
    
     currentstate = analogRead(ENCA);
     if (currentstate < laststate + 2) {
       tick++;
     }
     if (currentstate > laststate - 2) {
       tick --;
     }
     float ticks_per_revo = 25;
     float ratio = tick/ticks_per_revo;
     float wheel_radius = 0.127; //m
     float encoder_dist = 2*3.14*wheel_radius*ratio;
     
    if (encoder_dist < goal_distance) {
      Serial.println("Driving foward...");
      if (i <= 100) {
        i = i + 5;
        md.setM1Speed(i); //BL
        md.setM2Speed(i); //FL
        md.setM3Speed(i); //FR
        md.setM4Speed(i); //BR
        stopIfFault();
      }
    }
    else {
      i = 0;
      md.setM1Speed(i); //BL
      md.setM2Speed(i); //FL
      md.setM3Speed(i); //FR
      md.setM4Speed(i); //BR
      count = 2; // change to 1 if you want repeatable waypoint nav from ros data
    }
    laststate = currentstate;
  }
}
