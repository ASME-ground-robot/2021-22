
#include "DualVNH5019MotorShieldMod3.h"


#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle nh;


int INA1 = 30; //FR
int INB1 = 32; //FR
int EN1DIAG1 = 28;
int CS1 = A0;
int INA2 = 24; //BR
int INB2 = 26; //BR
int EN2DIAG2 = 22;
int CS2 = A1;
int INA3 = 31; //FL
int INB3 = 33; //FL
int EN3DIAG3 = 29;
int CS3 = A2;
int INA4 = 25; //BL
int INB4 = 27; //BL
int EN4DIAG4 = 23;
int CS4 = A3;
int PWM1 = 9; //FR
int PWM2 = 5; //BR
int PWM3 = 6; //FL
int PWM4 = 3; //BL

int i;

DualVNH5019MotorShieldMod3 md(INA1, INB1, EN1DIAG1, CS1, INA2, INB2, EN2DIAG2, CS2, INA3, INB3, EN3DIAG3, CS3, INA4, INB4, EN4DIAG4, CS4, PWM1, PWM2, PWM3, PWM4);

void messageCb(const std_msgs::Int16MultiArray &msg)
{
  int var1 = msg.data[0];
  int var2 = msg.data[1];
  int var3 = msg.data[2];
  int var4 = msg.data[3];
  int var5 = msg.data[4];

  if (var1 == 1)
  {
    i = 100 * var5;
    md.setM1Speed(i);
    md.setM2Speed(i);
    md.setM3Speed(i);
    md.setM4Speed(i);
  } // Move Forward

  if (var2 == 1)
  {
    i = 100 * var5;
    md.setM1Speed(-i);
    md.setM2Speed(-i);
    md.setM3Speed(-i);
    md.setM4Speed(-i);
  } // Move Backward

  if (var3 == 1)
  {
    i = 100 * var5;
    md.setM1Speed(-i);
    md.setM2Speed(-i);
    md.setM3Speed(i);
    md.setM4Speed(i);
  } // Left

  if (var4 == 1)
  {
    i = 100 * var5;
    md.setM1Speed(i);
    md.setM2Speed(i);
    md.setM3Speed(-i);
    md.setM4Speed(-i);
  } // Right

  if (var1 == 0 && var2 == 0 && var3 == 0 && var4 == 0)
  {
    md.setM1Speed(0);
    md.setM2Speed(0);
    md.setM3Speed(0);
    md.setM4Speed(0);
  }


}

ros::Subscriber<std_msgs::Int16MultiArray> sub("py_control", &messageCb); //py_control is the topic name of the python file

void setup()
{

  Serial.begin(115200);
  Serial.println("Starting 2X Dual VNH5019 Motor Shield - Mod3 LIB");
  md.init(); //start the motor shields and motor pins

  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(200);
}
