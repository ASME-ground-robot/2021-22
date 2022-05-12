   1 /*
   2  * rosserial Servo Control Example
   3  *
   4  * This sketch demonstrates the control of hobby R/C servos
   5  * using ROS and the arduiono
   6  * 
   7  * For the full tutorial write up, visit
   8  * www.ros.org/wiki/rosserial_arduino_demos
   9  *
  10  * For more information on the Arduino Servo Library
  11  * Checkout :
  12  * http://www.arduino.cc/en/Reference/Servo
  13  */
  14 
  15 #if defined(ARDUINO) && ARDUINO >= 100
  16   #include "Arduino.h"
  17 #else
  18   #include <WProgram.h>
  19 #endif
  20 
  21 #include <Servo.h> 
  22 #include <ros.h>
  23 #include <std_msgs/UInt16.h>
  24 
  25 ros::NodeHandle  nh;
  26 
  27 Servo servo;
  28 
  29 void servo_cb( const std_msgs::UInt16& cmd_msg){
  30   servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
  31   digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
  32 }
  33 
  34 
  35 ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);
  36 
  37 void setup(){
  38   pinMode(13, OUTPUT);
  39 
  40   nh.initNode();
  41   nh.subscribe(sub);
  42   
  43   servo.attach(9); //attach it to pin 9
  44 }
  45 
  46 void loop(){
  47   nh.spinOnce();
  48   delay(1);
  49 }
