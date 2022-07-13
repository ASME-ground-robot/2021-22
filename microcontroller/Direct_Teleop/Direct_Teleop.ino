#include <CustomDualG2HighPowerMotorShield.h>
DualG2HighPowerMotorShield18v22 md;

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle nh;

const int LED1 = 53; //Teleop LED-> ON
int i = 0;

void messageCb(const std_msgs::Int16MultiArray &msg) {
  
  //USB joystick config
  int foward = msg.data[0];
  int reverse = msg.data[1];
  int left = msg.data[2];
  int right = msg.data[3];
  int vel = msg.data[6];

  if (foward == 1) {
    md.enableDrivers();
    delay(1);
    i = 100*vel;
    md.setM1Speed(i); //BL
    md.setM2Speed(i); //FL
    md.setM3Speed(i); //FR
    md.setM4Speed(i); //BR
    stopIfFault();
  }
  else if (reverse == 1) {
    md.enableDrivers();
    delay(1);
    md.setM1Speed(-i); //BL
    md.setM2Speed(-i); //FL
    md.setM3Speed(-i); //FR
    md.setM4Speed(-i); //BR
    stopIfFault();
  }
  else if (left == 1) {
    md.enableDrivers();
    delay(1);
    i = 100*vel;
    md.setM1Speed(-i); //BL
    md.setM2Speed(-i); //FL
    md.setM3Speed(i); //FR
    md.setM4Speed(i); //BR
    stopIfFault();
  }
  else if (right == 1) {
    md.enableDrivers();
    delay(1);
    i = 100*vel;
    md.setM1Speed(i); //BL
    md.setM2Speed(i); //FL
    md.setM3Speed(-i); //FR
    md.setM4Speed(-i); //BR
    stopIfFault();
  }
  else if (foward != 1 && reverse != 1 && left != 1 && right != 1) {
    i = 0;
    md.setM1Speed(i); //BL
    md.setM2Speed(i); //FL
    md.setM3Speed(i); //FR
    md.setM4Speed(i); //BR
    
    md.disableDrivers();
    delay(1);
  }
  if (foward == 1 || reverse == 1 || left == 1 || right == 1) {
     digitalWrite(LED1, HIGH);
  }
}

//py_control is the topic name of the publisher
ros::Subscriber<std_msgs::Int16MultiArray> sub("py_control", &messageCb);

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
  
  pinMode(LED1, OUTPUT);
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
}
