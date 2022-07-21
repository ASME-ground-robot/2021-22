# 2021-22 version of software for the Mars Rover Senior Design Project at Cal State LA (Robotics Laboratory, ASME Student Org)
How to edit markdown files - https://www.markdownguide.org/cheat-sheet

# Contents
 - [About](#about)
 - [What's included in the package?](#package)
 - [Introduction of the robot](#introduction-of-the-robot)
 - [Software organization](#software-organization)
 - [Simulation](#simulation)
 - - [Autonomous Navigation](#autonomous-navigation)
 - - [Making a Gazebo World](#making-a-gazebo-world)
 - - [Robotic Arm](#robotic-arm)
 - - [Operating the Robotic Arm](#operating-the-robotic-arm)
 - [Installation instructions](#installation-instructions)


# About
In collaboration with Cal State LA's Robotics Laboratory and ASME Student Org, our rover "Cali" is a multi-year senior design project built for the University Rover Challenge held annually in Utah. Both hardware and software were utilized to create Cali such as SolidWorks, Arduinos, and a Jetson TX2. Our framework is based on ROS using Ubuntu 18.04. 

![Rover Overview](https://github.com/CSULA-URC/2021-22/blob/main/doc/Rover%202.JPG)
![Drive System](https://github.com/CSULA-URC/2021-22/blob/main/doc/Drive%20System%202.JPG)
![Suspension Bracket](https://github.com/CSULA-URC/2021-22/blob/main/doc/Suspension%20Bracket%202.JPG)
![Robotic Arm Use](https://github.com/CSULA-URC/2021-22/blob/main/doc/roboticarmgif3.gif)
![Teleoperation](https://github.com/CSULA-URC/2021-22/blob/main/doc/rovergif1.gif)


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
All codes used to control hardware for the rover. Contains important microcontroller codes and libraries as well as codes for teleoperation using joystick, autonomous navigation, sensor data integration, computer vision, and necessary ROS files.
  
![ECST Lab](https://github.com/CSULA-URC/2021-22/blob/main/doc/ecst_lab_world.png)
![Sandbox](https://github.com/CSULA-URC/2021-22/blob/main/doc/Sand_box_world.png)
![Wooden Box](https://github.com/CSULA-URC/2021-22/blob/main/doc/Wooden_box_world.png)
  #### Making a Gazebo World
  To launch an empty world in gazebo:
   - open a terminal 
   - type gazebo
   - press the key enter, this will open the Gazebo application with an empty world environment 

![Gazebo Instructions 1](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct1.jpg)
  
Next on the top left, access the edit category and select Building Editor
  
![Gazebo Instructions 2](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct2.jpg)
  
This should show below: 
 
![Gazebo Instructions 3](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct3.jpg)

NOTE: This tutorial assumes the user has a “blueprint” of the model. If not, then the user can customize to their needs as shown on the left.  

Using a blueprint image of a map, the user can import it to show on the white square grid above. 
Note that importing an image is only supported by the following formats: .jpg and .png

![Gazebo Instructions 4](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct4.jpg)

Once imported, a scale of the map is needed for proportional consistency of the real-world to the simulations. 

![Gazebo Instructions 5](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct5.jpg)
![Gazebo Instructions 6](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct6.jpg)

Now, simply trace the obstacles on the white grid with the wall option. This represents the areas where laser scans would bounce to emit back data. 

Once the map environment is traced with walls, the user has an option to decorate the barriers with color or visual textures such as a brick wall. 

![Gazebo Instructions 7](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct7.jpg)
![Gazebo Instructions 8](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct8.jpg)
![Gazebo Instructions 9](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct9.jpg)
![Gazebo Instructions 10](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct10.jpg)

To save the model, access the file category on the top left and provide a name of choice including the directory

Note: this only saves the environment as a model and not a world. So be careful to leave the pop up window as the model cannot be edited once exiting. 

This saves the model in the building editor folder directory but can be customized to save in whichever folder desired

![Gazebo Instructions 11](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct11.jpg)

To include items such as a coke can, access the insert category and from the huge number of options, place items by dragging the option to the world environment. 

![Gazebo Instructions 12](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct12.jpg)

To save a world, access the file category and select save world as 
save the end of your file as .world 

![Gazebo Instructions 13](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct13.jpg)
![Gazebo Instructions 14](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_instruct14.jpg)




  ### Robotic Arm
Packages from ROS used to control the robotic arm using simulations. Contains microcontroller code, Moveit (Rviz) simulation packages, and Arm URDF model

![Gazebo Model](https://github.com/CSULA-URC/2021-22/blob/main/doc/gazebo_model.png)

  ### Operating the Robotic Arm

Directories: 
ScorboTesting/src/scorbot_movelt/launch
-	ScorboTesting is the name of the workspace 
-	Scorbot_moveit is the name of the package 

Steps: 
1.	Arduino to Jetson 
Open the Arduino Application Software
Verify and upload the code file: R.A.6MotorPID

![Robotic Arm Instructions 1](https://github.com/CSULA-URC/2021-22/blob/main/doc/roboticarm1.jpg)

2.	Rosserial 

Connects ROS to Arduino using rosserial
```
  rosrun rosserial_python serial_node.py_port:=/dev/ttyACM0_baud:=115200
```

3.	Open a new terminal 

Go to ScorboTesting 
```
  source devel/setup.bash
  roslaunch scorbot_moveit demo.launch 
```
Before moving the robotic arm through simulation, make sure to manually position the arm close to #^o and the end effector slightly pointing up. An example of this is shown below: 

![Robotic Arm Instructions 2](https://github.com/CSULA-URC/2021-22/blob/main/doc/roboticarm2.jpg) ![Robotic Arm Instructions 3](https://github.com/CSULA-URC/2021-22/blob/main/doc/roboticarm3.jpg)

![Robotic Arm Instructions 4](https://github.com/CSULA-URC/2021-22/blob/main/doc/roboticarm4.jpg)

Motion the ball located in between the gears of the end-effector

![Robotic Arm Instructions 5](https://github.com/CSULA-URC/2021-22/blob/main/doc/roboticarm5.jpg) ![Robotic Arm Instructions 5](https://github.com/CSULA-URC/2021-22/blob/main/doc/roboticarm5.jpg)

Planning and executing poses are now permissible 



## Installation instructions

  1. Open up a terminal on your linux machine.
  2. mkdir -p ~/catkin_ws/src (some simulations/packages in the folder will need to be recreated because of workspace name change)
  3. cd ~/catkin_ws/
  4. catkin_make
  5. cd ~/catkin_ws/src
  6. git clone https://github.com/j-505/2021-22.git
  7. cd ~/catkin_ws/
  8. source devel/setup.bash
  
