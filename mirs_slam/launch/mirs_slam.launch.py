import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    # Declare arguments #
    lidar_port = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0',
        description='Set lidar usb port.')

    # Launch files and Nodes #
    tf2_ros_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        parameters=[
                {'frame_id': 'base_link'},
                {'child_frame_id': 'laser'},
                {'x': 0.0},
                {'y': 0.0},
                {'z': 0.35},
                {'yaw': 1.57},
                {'pitch': 0.0},
                {'roll': 0.0},]
    )

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sllidar_ros2'),
            'launch'),
            '/sllidar_s1_launch.py']),
        launch_arguments={'serial_port': lidar_port}.items(),
        condition=LaunchConfigurationEquals('lidar', 'sllidar')
    )

    ld = LaunchDescription()
    ld.add_action(lidar_port)

    ld.add_action(sllidar_launch)
    ld.add_action(tf2_ros_node)

    return ld