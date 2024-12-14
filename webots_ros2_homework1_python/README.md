# Project 3 Launch Notes
Please note some files are named after files used in previous assignments.
The controller file is still Assignment4.py since it is the same one we used 
during the competition

### TO INSTALL PACKAGE FOR ASSIGNMENT 

1. Set up environment variables for ROS. 
   
source /opt/ros/humble/setup.bash

2. Fork your own repository of tbtrueblood_f24_robotics (using web interface)

3. Clone your fork

git clone https://www.github.com/tbtrueblood/tbtrueblood_f24_robotics


4. Make the package (for python, it really just installs the files

cd tbtrueblood_f24_robotics
colcon build


5. Set up variables to use the package you just created
   
source install/setup.bash

7. Start webots simulation with connect back to ROS in the virtual machine
   
ros2 launch webots_ros2_homework1_python f23_robotics_1_launch.py



### TEST THE CONNECTION BETWEEN ROS2 AND WEBOTS

Test the connection between webots and ROS, use a ROS based ASCII keyboard to move the simulated robot in Webots

1. Open another terminal

2. Redo the source commands 
   
source /opt/ros/humble/setup.bash
source install/setup.bash

### RUN CONTROLLER

ros2 run webots_ros2_homework1_python webots_ros2_homework1_python


