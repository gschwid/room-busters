# In your custom launch file (e.g., my_robot_mapping.launch.py)
import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    map_file_path = PathJoinSubstitution(
        [FindPackageShare('room-busters'), 'maps', 'rtabmap.db']
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare('room-busters'), 'rviz', 'occupancy_config.rviz']
    )


    return launch.LaunchDescription([
        Node(
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
                {"use_sim_time": True}
            ],
            remappings=[
                ("/rgb/image", "/camera/image_raw"),
                ("/rgb/camera_info", "/camera/camera_info"),
                ("/imu", "/DONTLOOK"), # It was auto subscribing to the IMU when the proper transformations werent set up.
                ("/scan", "/scan"),
                ("/odom", "/odom")
            ],
            output='screen',
        ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            parameters=[
                {"frame_id": "base_link"},
                {"odom_frame_id": "odom"},
                {"subscribe_scan": True},
                {"subscribe_depth": False},
                {"subscribe_stereo": False},
                {"subscribe_rgbd": False},
                {"queue_size": 10},
                {"approx_sync": True},
                {"use_sim_time": True}
            ],
            remappings=[
                ("/rgb/image", "/camera/image_raw"),
                ("/rgb/camera_info", "/camera/camera_info"),
                ("/imu", "/DONTLOOK"), # It was auto subscribing to the IMU when the proper transformations werent set up.
                ("/scan", "/scan"),
                ("/odom", "/odom")
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_display',
            arguments=['-d', rviz_config], 
            output='screen'
        )

    ])