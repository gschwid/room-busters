import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription# this is garbage needed for the wrold
from launch.launch_description_sources import PythonLaunchDescriptionSource # this is garbage needed for the wrold
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Set TURTLEBOT3_MODEL if not set
    if 'TURTLEBOT3_MODEL' not in os.environ:
        os.environ['TURTLEBOT3_MODEL'] = 'waffle'

    # Path to turtlebot3_gazebo package
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    launch_file_dir = os.path.join(turtlebot3_gazebo_dir, 'launch')

    map_file_path = PathJoinSubstitution(
        [FindPackageShare('room-busters'), 'maps', 'rtabmap.db']
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare('room-busters'), 'rviz', 'occupancy_config.rviz']
    )
    
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

    # Node to control SLAM
    slam_node = Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[
                {"frame_id": "base_link"},
                {"odom_frame_id": "odom"},
                {"database_path": map_file_path}, # Uses already mapped data.
                {"subscribe_depth": False},
                {"subscribe_scan": True},
                {"queue_size": 30},
                {"approx_sync": True},
                {"use_sim_time": True},
                {"Mem/IncrementalMemory": "false"},
                {"Mem/InitWMWithAllNodes": "true"}, # Load the map from the database right away.
                {"RGBD/StartAtOrigin": "true"},
                {"Rtabmap/LoopThr": "0.5"}, # Lowered to make more loop closures happen.
                {"RGBD/OptimizeMaxError": "0"},
                {"Kp/MaxFeatures": "1000"},
                {"Reg/Strategy": "1"}, # Changed to ICP for proximity detection since we are using lidar data          
                {"RGBD/NeighborLinkRefining": "true"}, 
                {"RGBD/ProximityBySpace": "true"},  
                {"RGBD/AngularUpdate": "0.01"},     
                {"RGBD/LinearUpdate": "0.01"},      
            ],
            remappings=[
                ("/rgb/image", "/camera/image_raw"),
                ("/rgb/camera_info", "/camera/camera_info"),
                ("/imu", "/DONTLOOK"), # It was auto subscribing to the IMU when the proper transformations werent set up.
                ("/scan", "/scan"),
                ("/odom", "/odom")
            ],
            output='screen',
        )
    
    # Node to control RViz
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_display',
            arguments=['-d', rviz_config], 
            output='screen'
        )

    ld = LaunchDescription()
    ld.add_action(turtlebot3_sim)
    ld.add_action(dumb_vacuum)
    ld.add_action(slam_node)
    ld.add_action(rviz_node)

    return ld
