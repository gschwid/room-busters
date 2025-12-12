# Final project for CSCI 4551

## Todo
tutorial for getting running with wsl 

https://docs.ros.org/en/iron/Tutorials/Advanced/Simulators/Webots/Installation-Windows.html

- Implement a way to give the robot a destination coordinate to traverse too.
- Figure out how to get map of the envoirnment in simulation (fed to path planner?)


- If time allows, SLAM the envoirnment to get a map of it. 


Phase 2:
- Implement the RRT* algorithm to find a path to the goal object.
- Create velocity commands from the given path found by RRT*.
## Useful resources
https://github.com/Minipada/gazebo_ros_2d_map


This also serves as our documentation. 

To start. We are running with different environments which is a first start place, getting environment configured. Grant is fancy and dual booting, and has ROS installed from his previous research experience. Henry, in a blunder downloaded it using WSL2, and also plans to use lab machines. Corey is planning to use the lab machines!

turtlebot waffle

how to run:

colcon build

source /opt/ros/jazzy/setup.bash

source install/setup.bash

ros2 launch room-busters turtlebot_launch.py



see a topic 
    ros2 topic echo /scan
//run teleop with
    ros2 run rqt_console rqt_console

//to run rviz
sudo apt install ros-jazzy-rviz2
ros2 run rviz2 rviz2

set the fixed frame to baselink
and add the topic laserscan from topic 
killall -9 ruby

## How to run SLAM stuff

Hopefully this works. Essentially I set up an already existing map, and have RTABMAP set to localize the turtlebot using this map. This is how you run it

Dependencies...

sudo apt install ros-jazzy-rtabmap-ros

In one terminal...

Navigate to your workspace

colcon build (This only needs to be ran once)

source install/local_setup.bash
ros2 launch room-busters enviornment_launch.py

In another terminal...

Navigate to your workspace

source install/local_setup.bash
ros2 launch room-busters slam_launch.py

If it works you should see an occupancy grid with the turtlebot TF frames in it (fingers crossed)

To get occupancy grid data, sub to the /map topic

