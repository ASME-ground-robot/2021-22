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


void messageCb(const std_msgs::Int16MultiArray &msg)
{
  // assuming data.axes are mapped between -1.0 and 1.0

  if (b[1] > DZL) 			// Forward
  {
    R =  100*b[1]*b[4];
    L = 100*b[1]*b[4];
    if (b[0] < -DZL)
    {
    R = R - R*b[0];
    }
    if (b[0] > DZL)
    {
    L = L - L*b[0];
    }
    md.setM1speed(R);
    md.setM2speed(R);
    md.setM3speed(-L);
    md.setM4speed(-L);
  }


  if (b[1] < DZL)     //Backward
  {
    R =  100*b[1]*b[4];
    L = 100*b[1]*b[4];
    if (b[0] < -DZL)
    {
    R = R + R*b[0];
    }
    if (b[0] > DZL)
    {
    L = L + L*b[0];
    }
    md.setM1speed(R);
    md.setM2speed(R);
    md.setM3speed(-L);
    md.setM4speed(-L);
  }


  if (b[1] < DZL && b[1] > -DZL && b[0] > DZL)
  {
    R = 100*b[0]*b[4];
    L = 100*b[0]*b[4];
    md.setM1speed(R);
    md.setM2speed(R);
    md.setM3speed(L);
    md.setM4speed(L);
  }

  if (b[1] < DZL && b[1] > -DZL && b[0] < -DZL)
  {
    R = 100*b[0]*b[4];
    L = 100*b[0]*b[4];
    md.setM1speed(-R);
    md.setM2speed(-R);
    md.setM3speed(-L);
    md.setM4speed(-L);
  }

  if (b[1] > DZL || b[1] < -DZL || b[0] > DZL || b[0] < -DZL ) 
  {
      digitalWrite(LED1,HIGH);
  }
}

//py_control is the topic name of the publisher
ros::Subscriber<std_msgs::Int16MultiArray> sub("py_control", &messageCb);



void setup() {
  Serial.begin(115200);
  md.init();
  pinMode(LED1, OUTPUT);
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
}




