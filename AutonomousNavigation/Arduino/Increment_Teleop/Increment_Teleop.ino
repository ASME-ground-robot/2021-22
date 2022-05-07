#include <CustomDualG2HighPowerMotorShield.h>
DualG2HighPowerMotorShield18v22 md;

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle nh;

const int LED1 = 53; //Teleop LED-> ON
int i = 0;

// Where Joystick Data will be received to control motors
void messageCb(const std_msgs::Int16MultiArray &msg) {
  
  //USB joystick config
  int foward = msg.data[0];
  int reverse = msg.data[1];
  int left = msg.data[2];
  int right = msg.data[3];
  int skidleft = msg.data[4];
  int skidright = msg.data[5];
  int vel = msg.data[6];

  if (foward == 1) {
    md.enableDrivers();
    delay(1);
    if (i <= 100*vel) {
      i = i + 15;
      md.setM1Speed(i); //BL
      md.setM2Speed(i); //FL
      md.setM3Speed(i); //FR
      md.setM4Speed(i); //BR
      stopIfFault();
    }
  }
  else if (reverse == 1) {
    md.enableDrivers();
    delay(1);
    if (i <= 100*vel) {
      i = i + 15;
      md.setM1Speed(-i); //BL
      md.setM2Speed(-i); //FL
      md.setM3Speed(-i); //FR
      md.setM4Speed(-i); //BR
      stopIfFault();
    }
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
  }
  /*
  else if (skidleft == 1) {
    md.enableDrivers();
    delay(1);
    if (i <= 100*vel) {
      i = i + 15;
      md.setM1Speed(-i/3); //BL
      md.setM2Speed(-i/3); //FL
      md.setM3Speed(i); //FR
      md.setM4Speed(i); //BR
      stopIfFault();
    }
  }
  else if (skidright == 1) {
    md.enableDrivers();
    delay(1);
    if (i <= 100*vel) {
      i = i + 15;
      md.setM1Speed(i); //BL
      md.setM2Speed(i); //FL
      md.setM3Speed(-i/3); //FR
      md.setM4Speed(-i/3); //BR
      stopIfFault();
    }
  }
  */
  else if (foward != 1 && reverse != 1 && left != 1 && right != 1 && skidleft != 1 && skidright != 1) {
    i = 0;
    md.setM1Speed(i); //BL
    md.setM2Speed(i); //FL
    md.setM3Speed(i); //FR
    md.setM4Speed(i); //BR
    
    md.disableDrivers();
    delay(1);
  }
 // Teleop LED On
  if (foward == 1 || reverse == 1 || left == 1 || right == 1 || skidleft == 1 || skidright == 1) {
     digitalWrite(LED1, HIGH);
  }
}

//py_control is the topic name where joystick data is sent from
ros::Subscriber<std_msgs::Int16MultiArray> sub("py_control", &messageCb);

// checks to see if motors are running correctly, otherwise turns them off
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
