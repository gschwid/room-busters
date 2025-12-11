# In your custom launch file (e.g., my_robot_mapping.launch.py)
import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[
                # Set parameters here or point to .ini file
                {"frame_id": "base_link"},
                {"odom_frame_id": "odom"},
                #{"database_path": "/workspace"}, # For saving/loading
                {"subscribe_depth": False},
                {"subscribe_scan": True},
                {"queue_size": 30},
                {"approx_sync": True},
                {"use_sim_time": True}
            ],
            # Remap topics to your robot's actual topics
            remappings=[
                ("/rgb/image", "/camera/image_raw"),
                ("/rgb/camera_info", "/camera/camera_info"),
                ("/imu", "/DONTLOOK"),
                ("/scan", "/scan"),
                ("/odom", "/odom")
            ],
            output='screen',
        ),
    ])