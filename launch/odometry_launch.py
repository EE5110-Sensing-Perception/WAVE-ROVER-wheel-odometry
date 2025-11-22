from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('odom_wheel')
    param_file = os.path.join(pkg_share, 'config', 'robot_params.yaml')

    odom_node = Node(
        package='odom_wheel',
        executable='odometry_estimator',
        name='odometry_estimator',
        output='screen',
        parameters=[param_file]
    )

    return LaunchDescription([
        odom_node
    ])
