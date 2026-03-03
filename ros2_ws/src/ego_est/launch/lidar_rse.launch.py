from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # gimbal ctrl driver
    gimbal_ctrl_driver_node = Node(
        package='a8_gimbal',
        executable='gimbal_ctrl_driver_node',
        name='gimbal_ctrl_driver',
        output='screen',
    )
    
    # camera
    gimbal_cam_node = Node(
        package='a8_gimbal',
        executable='gimbal_cam_node',
        name='gimbal_cam',
        output='screen',
    )
    
    # gimbal ctrl interfacing example

    enable_auto_trajectory = False
    gimbal_ctrl_server_node = Node(
        package='a8_gimbal',
        executable='gimbal_ctrl_server_node',
        name='gimbal_ctrl_server',
        output='screen',
        parameters=[{
            'enable_auto_trajectory': enable_auto_trajectory
        }]
    )    
    
    return LaunchDescription([
        gimbal_ctrl_driver_node,
        # gimbal_ctrl_server_node,
        gimbal_cam_node,
    ])
