"""
Launch file for VQA system

Launches the VQA node with configured parameters.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for VQA system."""
    
    # Declare launch arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera topic to subscribe to'
    )
    
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='Salesforce/blip2-opt-2.7b',
        description='BLIP-2 model name from HuggingFace'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='auto',
        description='Device to run model on: auto, cpu, or cuda'
    )
    
    # VQA Node
    vqa_node = Node(
        package='robot_vqa',
        executable='vqa_node',
        name='vqa_node',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'model_name': LaunchConfiguration('model_name'),
            'device': LaunchConfiguration('device'),
            'image_buffer_size': 10,
            'service_name': 'ask_question',
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        camera_topic_arg,
        model_name_arg,
        device_arg,
        vqa_node,
    ])
