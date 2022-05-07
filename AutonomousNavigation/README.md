# Autonomous Navigation Codes
## 1. Arduino
Codes to be uploaded to the Arduino Mega microcontroller on the rover to control the wheels
- Increment_Teleop - main code for teleoperation, provides gradual speed increase on motor wheels
- PID_test - test code for using PID controller to control motor speeds
- Sensor_Codes - All codes used to get sensor data information with integration to ROS
- Twist_Teleop - motor speed control using ROS Twist messages; used for both Telop and Auto
- auto_test - testing code for simple waypoint navigation and ROS integration
## 2. Computervision
AR tag image recognition using OpenCV.
- cv - python code to detect AR tags used in URC using OpenCV tag library
## 3. MotorDriver
Joystick codes used for Teleoperation
- joystick_teleop - codes for manual control using generic joystick controllers
- ps3joystick_teleop - code for manual control using ps3 controller connected wirelessly through bluetooth.
## 4. Simple_SM
- State_Machine - State machine that runs all other states in folder
- Initialize_Rover - State that checks if sensor data and other factors are received before starting other states
- Subscriber - code where sensor dat is called into other states
- Waypoint_Navigation - Main state that converts GPS/IMU/MAGNETOMETER data into distance and angle and sends to microcontroller
- check_output - State that checks outpout of waypoint navigation state
## 5. Simulations
Rviz/Gazebo packages to simulate simple rover and environment
## 6. StateMachines
Template nodes to be used for Autonomous Navigation
## 7. launch
Launch files to run every code for specific tasks (teleop, autonav, etc.)
## 8. msg
Message files to read sensor data
