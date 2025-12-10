import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription# this is garbage needed for the wrold
from launch.launch_description_sources import PythonLaunchDescriptionSource # this is garbage needed for the wrold
from launch_ros.actions import Node

def generate_launch_description():
    # Set TURTLEBOT3_MODEL if not set
    if 'TURTLEBOT3_MODEL' not in os.environ:
        os.environ['TURTLEBOT3_MODEL'] = 'waffle'

    # Path to turtlebot3_gazebo package
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    launch_file_dir = os.path.join(turtlebot3_gazebo_dir, 'launch')
    
    # This is from the internet, i don't know if its working properly, when i change it to house 
    #it doesn't work.
    turtlebot3_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'turtlebot3_world.launch.py')
        )
    )

    # Node to control the robot
    dumb_vacuum = Node(
        package='room-busters',
        executable='dumb_vacuum_v2',
        name='dumb_vacuum_v2',
    )

    ld = LaunchDescription()
    ld.add_action(turtlebot3_sim)
    ld.add_action(dumb_vacuum)

    return ld
