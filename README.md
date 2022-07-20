# About
In collaboration with Cal State LA's Robotics Laboratory and ASME Student Org, our rover "Cali" is a multi-year senior design project built for the University Rover Challenge held annually in Utah. Both hardware and software were utilized to create Cali such as SolidWorks, Arduinos, and a Jetson TX2. Our framework is based on ROS using Ubuntu 18.04. 
![Screenshot](Rover 2.JPG)

# 2021-22 version of software for the Mars Rover Senior Design Project at Cal State LA (Robotics Laboratory, ASME Student Org)
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
  - All codes used to control hardware for the rover. Contains important microcontroller codes and libraries as well as codes for teleoperation using joystick, autonomous navigation, sensor data integration, computer vision, and necessary ROS files.

  ### Robotic Arm
  - Packages from ROS used to control the robotic arm using simulations. Contains microcontroller code, Moveit (Rviz) simulation packages, and Arm URDF model

## 4. Installation instructions

  1. Open up a terminal on your linux machine.
  2. mkdir -p ~/catkin_ws/src (some simulations/packages in the folder will need to be recreated because of workspace name change)
  3. cd ~/catkin_ws/
  4. catkin_make
  5. cd ~/catkin_ws/src
  6. git clone https://github.com/j-505/2021-22.git
  7. cd ~/catkin_ws/
  8. source devel/setup.bash
