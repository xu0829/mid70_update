import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 获取 livox_ros2_driver 的 launch 文件路径
    livox_launch_dir = os.path.join(get_package_share_directory('livox_ros2_driver'), 'launch')
    
    # 包含 livox_lidar_rviz_launch.py
    livox_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_launch_dir, 'livox_lidar_rviz_launch.py')
        )
    )

    # 启动 roi_measure_node
    roi_measure_node = Node(
        package='roi_measure_tools',
        executable='roi_measure_node',
        name='roi_measure_node',
        output='screen'
    )

    # 启动 roi_box_server (推测你说的第二个应该是 roi_box_server)
    roi_box_server = Node(
        package='roi_measure_tools',
        executable='roi_box_server',
        name='roi_box_server',
        output='screen'
    )

    return LaunchDescription([
        livox_rviz_launch,
        roi_measure_node,
        roi_box_server,
    ])