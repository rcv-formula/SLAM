import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 설정 변수 정의
    config_dir = "/carto_ws/src/cartographer_ros/cartographer_ros/configuration_files"
    rosbag_file = LaunchConfiguration('bagfiles')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # cartographer_node 정의 및 변수에 할당
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[
            {"use_sim_time": use_sim_time},
            {"provide_odom_frame": True},
            {"use_odometry": False},
            {"publish_frame_projected_to_2d": True}
        ],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'Damvi_carto_config.lua',
        ],
        remappings=[
            ('scan', 'scan'),
            ('imu', 'imu/data'),
            ('tf', 'tf'),
            ('tf_static', 'tf_static'),
            ('odom', 'odom'),
        ],
    )
    # trajectory_to_odom 노드 추가
    trajectory_to_odom_node = Node(
        package='cartographer_ros',
        executable='trajectory_to_odom',
        name='trajectory_to_odom',
        output='screen',
        parameters=[
            {"use_sim_time": use_sim_time}
        ],
    )

    # Launch 설명 반환
    return LaunchDescription([
        # ROS bag 파일 설정
        DeclareLaunchArgument(
            'bagfiles',
            default_value='/rosbag/3F_slow/3F_slow_0.db3',
            description='Path to the rosbag file'
        ),

        # use_sim_time 설정
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),

        # ROS bag 재생 노드
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', rosbag_file, '--clock'],
            output='screen'
        ),

        # Cartographer Node 실행
        cartographer_node,

        # TrajectoryToOdom Node 실행
        trajectory_to_odom_node,

        # Occupancy Grid Node 실행
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[
                {'resolution': 0.05},
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('map', 'map'),
                ('occupancy_grid', 'map'),
            ],
        ),
    ])
