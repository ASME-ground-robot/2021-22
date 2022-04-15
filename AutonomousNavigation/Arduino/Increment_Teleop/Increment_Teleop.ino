#include <DualVNH5019MotorShieldMod3.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle nh;

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

const int LED1 = 11; //Teleoperation Mode LED-> ON
int i = 0;


void messageCb(const std_msgs::Int16MultiArray &msg) {
  int foward = msg.data[0];
  int reverse = msg.data[1];
  int left = msg.data[2];
  int right = msg.data[3];
  int skidleft = msg.data[4];
  int skidright = msg.data[5];
  int vel = msg.data[6];

  if (foward == 1) {
    if (i <= 100*vel) {
      i = i + 2;
      md.setM1Speed(i); //BL
      md.setM2Speed(i); //FL
      md.setM3Speed(-i); //FR
      md.setM4Speed(-i); //BR
    }
  }
  else if (reverse == 1) {
    if (i <= 100*vel) {
      i = i + 2;
      md.setM1Speed(-i); //BL
      md.setM2Speed(-i); //FL
      md.setM3Speed(i); //FR
      md.setM4Speed(i); //BR
    }
  }
  else if (left == 1) {
    if (i <= 100*vel) {
      i = i + 5;
      md.setM1Speed(-i); //BL
      md.setM2Speed(-i); //FL
      md.setM3Speed(-i); //FR
      md.setM4Speed(-i); //BR
    }
  }
  else if (right == 1) {
    if (i <= 100*vel) {
      i = i + 5;
      md.setM1Speed(i); //BL
      md.setM2Speed(i); //FL
      md.setM3Speed(i); //FR
      md.setM4Speed(i); //BR
    }
  }
  else if (skidleft == 1) {
    if (i <= 100*vel) {
      i = i + 2;
      md.setM1Speed(i/2); //BL
      md.setM2Speed(i/2); //FL
      md.setM3Speed(-i); //FR
      md.setM4Speed(-i); //BR
    }
  }
  else if (skidright == 1) {
    if (i <= 100*vel) {
      i = i + 2;
      md.setM1Speed(i/2); //BL
      md.setM2Speed(i/2); //FL
      md.setM3Speed(-i); //FR
      md.setM4Speed(-i); //BR
    }
  }
  else if (foward == 0 && reverse == 0 && left == 0 && right == 0 && skidleft == 0 && skidright == 0) {
    i = 0;
    md.setM1Speed(i); //BL
    md.setM2Speed(i); //FL
    md.setM3Speed(i); //FR
    md.setM4Speed(i); //BR
  }
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("py_control", &messageCb); //py_control is the topic name of the python file



void setup() {
  Serial.begin(115200);
  md.init();
  pinMode(LED1, OUTPUT);
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
}



void loop() {
  nh.spinOnce();
}
