import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Tìm đường dẫn đến thư mục share của package
    pkg_share = FindPackageShare('pre_filtering_imu')
    
    # Đường dẫn mặc định tới file cấu hình YAML 
    default_config_file = PathJoinSubstitution([pkg_share, 'config', 'imu_prefilter.yaml'])

    # Khai báo các đối số (Launch Arguments) 
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the YAML config file'
    )

    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/imu/data',
        description='Input IMU topic name'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/imu/proposed',
        description='Output filtered IMU topic name'
    )

    imu_sampling_freq_arg = DeclareLaunchArgument(
        'imu_sampling_freq',
        default_value='200.0',
        description='IMU sampling frequency'
    )

    # Định nghĩa Node ROS 2 
    imu_prefilter_node = Node(
        package='pre_filtering_imu',      # Tương đương với pkg trong ROS 1 
        executable='imu_prefilter_node', # Tương đương với type trong ROS 1 
        name='imu_prefilter_node',       # Tên node 
        output='screen',                 # Xuất log ra màn hình 
        parameters=[
            LaunchConfiguration('config_file'), # Load toàn bộ tham số từ file YAML 
            {
                # Ghi đè tham số bằng giá trị từ đối số truyền vào 
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'imu_sampling_freq': LaunchConfiguration('imu_sampling_freq'),
            }
        ]
    )

    return LaunchDescription([
        config_file_arg,
        input_topic_arg,
        output_topic_arg,
        imu_sampling_freq_arg,
        imu_prefilter_node
    ])
