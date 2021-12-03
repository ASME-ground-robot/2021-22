# 2021-22 version of software for the Ground Robot at Cal State LA (Robotics Laboratory, ASME Student Org)
How to edit markdown files - https://www.markdownguide.org/cheat-sheet

## 1. Introduction of the robot  

- hardware - Jetson TX II, four individually controlled wheels, one robot arm (scorbot-er3u)  
- software - Ubuntu 18.04, ROS  
- sensors - IMU, GPS, Lidar, Antenna, Transceiver, Camera, Encoders

## 2. Software organization  

  ### Autonomous Navigation
  - State Machines
    - test
    - path planning
    - waypoint Navigation
  - Lidar
    - test
    - code
  - GPS
    - test
    - code
  - Computer Vision
    - test
    - code
  - IMU Sensor
    - test
    - code
  - Motor Driver
    - test
    - joystick
  ### Robotic Arm
  - Simulations
    - moveit
    - gazebo
    - moveittogazebo
  - Encoder
    -moveittoencoder
## 3. What's included in the package?

  This package is a detailed outline of the software files needed to run the Senior Design Mars Rover from California State University, Los Angeles.
  The package is dedicated to ROS 1 and the Ubuntu version is 18.04.
## 4. Installation instructions


  1. Open up a terminal on your linux machine.
  2. Go to your current workspace src folder
  3. cd your_ws/src
  4. git clone https://github.com/j-505/2021-22.git
  5. cd your_ws ..
  6. source devel/setup.bash
