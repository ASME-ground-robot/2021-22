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
int i;

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
    md.setM1Speed(i); //BL
    md.setM2Speed(i); //FL
    md.setM3Speed(-i); //FR
    md.setM4Speed(-i); //BR
  } // Move Forward

  if (var2 == 1)
  {
    i = 100 * var5;
    md.setM1Speed(-i);
    md.setM2Speed(-i);
    md.setM3Speed(i);
    md.setM4Speed(i);
  } // Move Backward

  if (var3 == 1)
  {
    i = 100 * var5;
    md.setM1Speed(-i);
    md.setM2Speed(-i);
    md.setM3Speed(-i);
    md.setM4Speed(-i);
  } // Left

  if (var4 == 1)
  {
    i = 100 * var5;
    md.setM1Speed(i);
    md.setM2Speed(i);
    md.setM3Speed(i);
    md.setM4Speed(i);
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
  //Serial.println("Starting 2X Dual VNH5019 Motor Shield - Mod3 LIB");
  md.init(); //start the motor shields and motor pins
  pinMode(LED1, OUTPUT);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

}

void loop()
{
  delay(100);
  nh.spinOnce();
}
