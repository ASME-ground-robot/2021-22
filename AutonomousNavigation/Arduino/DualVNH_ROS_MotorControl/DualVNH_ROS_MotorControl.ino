
#include "DualVNH5019MotorShieldMod3.h"


#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle nh;

// green
int INA1 = 31; //BL
int INB1 = 33; //BL
int EN1DIAG1 = 29;

//yellow
int INA2 = 25; //FL
int INB2 = 27; //FL
int EN2DIAG2 = 23;

//blue
int INA3 = 30; //FR
int INB3 = 32; //FR
int EN3DIAG3 = 28;

//orange
int INA4 = 24; //BR
int INB4 = 26; //BR
int EN4DIAG4 = 22;

int PWM1 = 12; //FR
int PWM2 = 3; //BR
int PWM3 = 9; //FL
int PWM4 = 5; //BL
const int LED1 = 11; //Teleoperation Mode LED-> ON
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
 if (var1 == 1 || var2 == 1 || var3 == 1 || var4 == 1)
  {
     digitalWrite(LED1,HIGH);
     }
 else 
     {
     digitalWrite(LED1, LOW);
  }

}

ros::Subscriber<std_msgs::Int16MultiArray> sub("py_control", &messageCb); //py_control is the topic name of the python file

void setup()
{

  Serial.begin(115200);
  Serial.println("Starting 2X Dual VNH5019 Motor Shield - Mod3 LIB");
  md.init(); //start the motor shields and motor pins
  pinMode(LED1, OUTPUT);
  
  nh.initNode();
  nh.subscribe(sub);

}

void loop()
{
  nh.spinOnce();
  delay(200);
}
