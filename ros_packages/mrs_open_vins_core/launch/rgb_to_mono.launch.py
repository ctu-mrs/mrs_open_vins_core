from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import UnlessCondition
import os

def generate_launch_description():
    # Declare launch arguments
    uav_name_arg = DeclareLaunchArgument(
        name='uav_name',
        default_value=os.environ["UAV_NAME"]
    )
    
    input_image_topic_arg = DeclareLaunchArgument(
        'input_image_topic',
        default_value=PathJoinSubstitution(["/", LaunchConfiguration('uav_name'), "image_raw"]),
        description='Input color image topic'
    )
    
    output_image_topic_arg = DeclareLaunchArgument(
        'output_image_topic',
        default_value=PathJoinSubstitution(["/", LaunchConfiguration('uav_name'), "image_mono"]),
        description='Output grayscale image topic'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value=PathJoinSubstitution(["/", LaunchConfiguration('uav_name'), "camera_info"]),
        description='Camera info topic'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='image_proc',
        description='Namespace for the image_proc components'
    )
    
    container_id_arg = DeclareLaunchArgument(
        'container_id',
        default_value='image_processing_container',
        description='Name of the component container'
    )
    
    standalone_arg = DeclareLaunchArgument(
        name='standalone',
        default_value='true'
    )
    
    # Get launch configuration values
    input_image_topic = LaunchConfiguration('input_image_topic')
    output_image_topic = LaunchConfiguration('output_image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    namespace = LaunchConfiguration('namespace')
    container_id = LaunchConfiguration('container_id')
    
    # Create composable node for image processing
    image_proc_component = ComposableNode(
        package='image_proc',
        plugin='image_proc::DebayerNode',
        name='color_to_grayscale',
        namespace=namespace,
        remappings=[
            ('image_raw', input_image_topic),
            ('image_mono', output_image_topic),
            ('camera_info', camera_info_topic),
        ],
        parameters=[
            {'queue_size': 10}
        ]
    )
    
    loader = LoadComposableNodes(
        condition=UnlessCondition(LaunchConfiguration('standalone')),
        composable_node_descriptions=[image_proc_component],
        target_container=LaunchConfiguration('container_id'),
    )
    
    # # Container to hold the component
    # container = ComposableNodeContainer(
    #     name=container_id,
    #     namespace='',
    #     package='rclcpp_components',
    #     executable='component_container',
    #     composable_node_descriptions=[
    #         image_proc_component,
    #     ],
    #     output='screen'
    # )
    
    return LaunchDescription([
        uav_name_arg,
        input_image_topic_arg,
        output_image_topic_arg,
        camera_info_topic_arg,
        namespace_arg,
        container_id_arg,
        standalone_arg,
        #container,
        loader
    ])