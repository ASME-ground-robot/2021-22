# 2021-22 version of software for the Mars Rover Senior Design Project at Cal State LA (Robotics Laboratory, ASME Student Org)
How to edit markdown files - https://www.markdownguide.org/cheat-sheet

# Contents
 - [About](#about)
 - [What's included in the package?](#package)
 - [Introduction of the robot](#introduction-of-the-robot)
 - [Software organization](#software-organization)
 - [Simulation](#simulation)
 - [Installation instructions](#installation-instructions)


# About
In collaboration with Cal State LA's Robotics Laboratory and ASME Student Org, our rover "Cali" is a multi-year senior design project built for the University Rover Challenge held annually in Utah. Both hardware and software were utilized to create Cali such as SolidWorks, Arduinos, and a Jetson TX2. Our framework is based on ROS using Ubuntu 18.04. 

![Rover Overview](https://github.com/CSULA-URC/2021-22/blob/main/doc/Rover%202.JPG)
![Drive System](https://github.com/CSULA-URC/2021-22/blob/main/doc/Drive%20System%202.JPG)
![Suspension Bracket](https://github.com/CSULA-URC/2021-22/blob/main/doc/Suspension%20Bracket%202.JPG)

## Package
- This package is a detailed outline of the software files needed to run the Senior Design Mars Rover from California State University, Los Angeles.
  The package is dedicated to ROS 1 and the Ubuntu version is 18.04.
  
## Introduction of the robot  

"Cali" is a rover redesigned from the University Rover Challenge to a paper "Low cost mobile robotic arm". 

The mechanical systems encompasses the chassis, suspension system, and drive system. The chassis consists of 1"x1" galvanized steel bolt together tubing for modularity. The suspension system consists of an independent radius arm setup with air shock absorbers for off road capabilities. The drive systems consists of 12V motors with 9" beach wheels for sandy and rocky terrain.

The electrical systems encompasses power, sensing, controls, and communications. A 12V 50ah li-on battery is used to power
- hardware - Jetson TX II, four individually controlled wheels, one robot arm (scorbot-er3u)  
- software - Ubuntu 18.04, ROS  
- sensors - IMU, GPS, Lidar, Antenna, Transceiver, Camera, Encoders

## Software organization  

Brief descriptions of each folder. For more detailed information, please look at folder's readme files. (Will create soon)

  - hector_slam: package that uses lidar laser data to scan an environment and create a grayscale 2d map
  - microcontroller: all arduino codes used to control the robot and arm
  - ros-imu-bno055: package that lets us read all imu data and represent it in rviz
  - rover: all misc. files worked on the robot like teleoperation, opencv, etc.
  - rover_autonav: all the autonomous navigation simulations used for this project
  - rplidar_ros: package to get the rplidara1 to work with ros
  - scorbot: visualization and controls of the robotic arm
  - shared_tavern: used as a sharing folder to share files
  - doc: screenshots and images

## Simulation
  ### Autonomous Navigation
  - All codes used to control hardware for the rover. Contains important microcontroller codes and libraries as well as codes for teleoperation using joystick, autonomous navigation, sensor data integration, computer vision, and necessary ROS files.
  
![ECST Lab](https://github.com/CSULA-URC/2021-22/blob/main/doc/ecst_lab_world.png)
![Sandbox](https://github.com/CSULA-URC/2021-22/blob/main/doc/Sand_box_world.png)
![Wooden Box](https://github.com/CSULA-URC/2021-22/blob/main/doc/Wooden_box_world.png)

  ### Robotic Arm
  - Packages from ROS used to control the robotic arm using simulations. Contains microcontroller code, Moveit (Rviz) simulation packages, and Arm URDF model

## Installation instructions

  1. Open up a terminal on your linux machine.
  2. mkdir -p ~/catkin_ws/src (some simulations/packages in the folder will need to be recreated because of workspace name change)
  3. cd ~/catkin_ws/
  4. catkin_make
  5. cd ~/catkin_ws/src
  6. git clone https://github.com/j-505/2021-22.git
  7. cd ~/catkin_ws/
  8. source devel/setup.bash
  
