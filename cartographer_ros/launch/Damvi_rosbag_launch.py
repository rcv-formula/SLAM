import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('f1tenth_cartographer')
    cartographer_dir = get_package_share_directory('cartographer_ros')

    # 경로 설정: ROS bag 파일 경로 지정
    rosbag_file = os.path.join(package_dir, 'data', 'example.bag')  # rosbag 파일 위치 지정

    return LaunchDescription([
        # Rosbag 파일 재생 노드
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', rosbag_file],
            output='screen'
        ),

        # Cartographer 노드
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[os.path.join(package_dir, 'config', 'cartographer_config.lua')],
            remappings=[
                ('/scan', '/scan'),
                ('/imu', '/imu/data'),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        ),

        # Cartographer occupancy grid 노드
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'resolution': 0.05}],
            remappings=[
                ('/map', '/map'),
                ('/occupancy_grid', '/map'),
            ],
        ),

        # RViz 시각화 노드
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(package_dir, 'config', 'cartographer.rviz')],
            output='screen',
        ),
    ])

