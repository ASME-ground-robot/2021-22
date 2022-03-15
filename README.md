# 2021-22 version of software for the Ground Robot at Cal State LA (Robotics Laboratory, ASME Student Org)
How to edit markdown files - https://www.markdownguide.org/cheat-sheet

## 1. What's included in the package?
- This package is a detailed outline of the software files needed to run the Senior Design Mars Rover from California State University, Los Angeles.
  The package is dedicated to ROS 1 and the Ubuntu version is 18.04.
  
## 2. Introduction of the robot  

- hardware - Jetson TX II, four individually controlled wheels, one robot arm (scorbot-er3u)  
- software - Ubuntu 18.04, ROS  
- sensors - IMU, GPS, Lidar, Antenna, Transceiver, Camera, Encoders

## 3. Software organization  

  ### Autonomous Navigation
  - Arduino
    - Autonav_driver
    - Teleop_MotorControl
  - Computer Vision
    - cv (OpenCV)
  - Motor Driver
    - joystick_teleop
  - Sensors
  - Simulations
    - mobile_rover_basic
  - State Machines
  - Launch
    - rover_teleop
  - Msg
  ### Robotic Arm
  - Simulations
    - moveit
    - gazebo
    - moveittogazebo
  - Encoder
    - moveittoencoder

## 4. Installation instructions

  1. Open up a terminal on your linux machine.
  2. mkdir -p ~/catkin_ws/src
  3. cd ~/catkin_ws/
  4. catkin_make
  5. cd ~/catkin_ws/src
  6. git clone https://github.com/j-505/2021-22.git
  7. cd ~/catkin_ws/
  8. source devel/setup.bash
