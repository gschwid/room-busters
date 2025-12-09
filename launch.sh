#!/usr/bin/bash

export TURTLEBOT3_MODEL=waffle
source /opt/ros/jazzy/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py