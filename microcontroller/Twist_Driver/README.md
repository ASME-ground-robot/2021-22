# How to Use:
Note: First download the packages: teleop_twist_joy & joy (to use with joystick), teleop_twist_keyboard (to use with keyboard), rqt_robot_steering, rosserial

- upload Twist_Driver to Arduino
- in terminal, run "roscore"
- in new terminal, run "rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200"
- in new terminal, start any of the packages downloaded depending on how u want to control it.
--ex: for rqt_robot_steering, run "rosrun rqt_robot_steering rqt_robot_steering"
- if u wanna see the speed values (in m/s) run "rostopic echo /cmd_vel" in new terminal

Note: dont test code while robot is on ground or else it might run off and crash unless you have a way to control the speed to stop it. test it by putting the robot on top of the wooden stands or something so the wheels can run freely in the air.
