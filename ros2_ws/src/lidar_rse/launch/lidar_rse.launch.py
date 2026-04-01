from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    config_file = os.path.join(
        get_package_share_directory('lidar_rse'),
        'launch',
        'rse_params.yaml'
    )
    
    lidar_rse_node = Node(
        package='lidar_rse',
        executable='lidar_rse_node',
        name='lidar_rse',
        output='screen',
        parameters=[
            config_file,
        ]
    )
    
    return LaunchDescription([
        lidar_rse_node,
        # gimbal_ctrl_server_node,
        # gimbal_cam_node,
    ])
