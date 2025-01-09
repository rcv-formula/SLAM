import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('cartographer_ros')
    config_dir = os.path.join(package_dir, 'configuration_files')  # 경로 수정

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments = [
                '-configuration_directory', config_dir,
                '-configuration_basename', 'Damvi_carto_config.lua'],
            remappings=[
                ('scan', 'scan'),
                ('imu', 'imu/data'),  # 데이터 스트림 이름 일치 확인 필요
                ('tf', 'tf'),
                ('tf_static', 'tf_static'),
            ],
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'resolution': 0.05}],
            remappings=[
                ('map', 'map'),
                ('occupancy_grid', 'map'),
            ],
        ),

        Node(
            package='cartographer_ros',
            executable='trajectory_to_odom',
            name='trajectory_to_odom',
            output='screen',
        )
    ])

