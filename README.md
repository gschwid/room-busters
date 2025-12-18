# Running Room Busters Project

### Dependencies

This project assumes you have both Gazeebo and ROS2 Jazzy installed. Only dependency beyond that is rtabmap, it can be installed with.

sudo apt install ros-jazzy-rtabmap-ros

Additionally, Henry experimented with using WSL, while it had it's issues (not being able to use the GPU due to package conflicts) it still worked well for not having to be in lab. Documentation for using ROS with WSL2 (and wslg) can be found here: https://docs.ros.org/en/iron/Tutorials/Advanced/Simulators/Webots/Installation-Windows.html


### What does our code do?

Displays the movement logic for Buster the robot when cleaning and exploring its envoirnment. It traverses the envoirnment by picking a direction, moving straight until it gets close to an obstacle, then rotating and moving a new random direction that contains no immediate obstacles. The robot uses lidar data to determine where the obstacles are around it. As it is cleaning the envoirnment, RTABMAP is used to localize itself within an existing map of the envoirnment.

While it is running you will see both Buster traversing the Gazeebo envoirnment, and Buster being localized in a occupancy grid.


### How to run it.

move the room-busters package into {ros2_ws}/src

cd {ros2_ws}

colcon build

If you must source ros2: source /opt/ros/jazzy/setup.bash

source install/local_setup.bash

ros2 launch room-busters demo_launch.py





