import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    script_path = os.path.abspath(__file__)
    main_dir = os.path.dirname(script_path)
    package_dir = os.path.dirname(main_dir)
    config_dir = os.path.join(package_dir, 'configuration_files')
    pbstream_file = os.path.join(package_dir, 'pbstream', '/rosbag/4f_map.pbstream')   # .pbstream 파일이 있는 위치로 경로 수정
    rosbag_file = LaunchConfiguration('bagfiles')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        
        DeclareLaunchArgument(
            'bagfiles',
            default_value='/rosbag/4f_2/4f_2_0.db3',    # rosbag 파일이 있는 위치로 경로 수정
            description='Path to the rosbag file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'Damvi_localization_config.lua',
                '-load_state_filename', pbstream_file 
            ],
            remappings=[
                ('scan', 'scan'),
                ('imu', 'imu/data'),
                ('tf', 'tf'),
                ('tf_static', 'tf_static'),
            ],
            parameters=[
                {"use_sim_time": use_sim_time},
                {"provide_odom_frame": True},
                {"use_odometry": False},
                {"publish_frame_projected_to_2d": True}
            ],
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[
                {'resolution': 0.05}
            
            ],
            remappings=[
                ('map', 'map'),
                ('occupancy_grid', 'map'),
            ],
        ),

        # Optional node to transform trajectory to odom for robot localization
        Node(
            package='cartographer_ros',
            executable='trajectory_to_odom',
            name='trajectory_to_odom_node',
            output='screen',
            parameters=[
               {"use_sim_time": True},
            ]
        ),

        # ROS bag 재생 노드
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', rosbag_file, '--clock'],
            output='screen'
        )
    ])
