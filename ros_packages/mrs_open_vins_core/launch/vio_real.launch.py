import yaml
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def get_processed_launch_objects(context):
    _custom_config_file = LaunchConfiguration('custom_config').perform(context)
    remappings = []
    executable_name = "run_subscribe_msckf"
    prefix = executable_name
    # pull remapping of the topics out of the yaml file
    if _custom_config_file != '':
        with open(_custom_config_file, 'r') as f:
            yaml_data = yaml.load(f, Loader=yaml.FullLoader)

            if prefix in yaml_data and 'remappings' in yaml_data[prefix]['ros__parameters']:
                remappings_subyaml = yaml_data[prefix]['ros__parameters']['remappings']
                for orig_name in remappings_subyaml:
                    new_name = remappings_subyaml[orig_name]
                    remappings.append((orig_name, new_name))
                    
    objects = [
        LogInfo(msg=f"custom config file: {_custom_config_file}"),
        LogInfo(msg=f"remappings:"),
    ]
        
    for remapping in remappings:
        objects.append(LogInfo(msg=f"\t{remapping[0]} -> {remapping[1]}"))
    
    objects.append(DeclareLaunchArgument(name='uav_name',               default_value=os.environ["UAV_NAME"]))
    # Construct the container ID
    objects.append(DeclareLaunchArgument(name='container_name',         default_value='vio_main_container'))
    objects.append(DeclareLaunchArgument(name='container_namespace',    default_value=LaunchConfiguration('uav_name')))
    objects.append(DeclareLaunchArgument(name='container_id',           default_value=PathJoinSubstitution([LaunchConfiguration('container_namespace'), LaunchConfiguration('container_name')])))
    
    objects.append(ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace=LaunchConfiguration('container_namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[],  # Start empty
        output='screen',
    ))
    
    # Conditionally include bluefox2 launch
    objects.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bluefox2'),
                'launch',
                'single.launch.py'
            ])
        ]),
        launch_arguments={
            'custom_config': LaunchConfiguration('custom_config'),
            'container_id': LaunchConfiguration('container_id'),
            'standalone': 'false'
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_bluefox_cam_and_imu'))
    ))
    
    # Conditionally include mrs_serial IMU launch
    objects.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mrs_serial'),
                'launch',
                'vio_imu.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_bluefox_cam_and_imu')),
        launch_arguments={
            #'custom_config': LaunchConfiguration('custom_config')
            'container_id': LaunchConfiguration('container_id'),
            'standalone': 'false'
        }.items()
    ))
    
    objects.append(DeclareLaunchArgument(name='config',            default_value='realworld_bluefox_front',  description=''))
    objects.append(DeclareLaunchArgument(name='verbosity',         default_value='DEBUG',       description='ALL, DEBUG, INFO, WARNING, ERROR, SILENT'))
    
    objects.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ov_msckf'),
                'launch',
                'subscribe_composable.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_bluefox_cam_and_imu')),
        launch_arguments={
            #'custom_config': LaunchConfiguration('custom_config')
            'container_id': LaunchConfiguration('container_id'),
            'standalone': 'false',
            'config_path': PathJoinSubstitution([
                FindPackageShare('mrs_open_vins_core'),
                'config',
                LaunchConfiguration('config'),
                'estimator_config.yaml'
            ]),
            'custom_config': _custom_config_file
        }.items(),
    ))
    
    objects.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mrs_vins_imu_filter'),
                'launch',
                'filter_icm_42688.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_bluefox_cam_and_imu')),
        launch_arguments={
            #'custom_config': LaunchConfiguration('custom_config')
            'container_name': LaunchConfiguration('container_id'),
            'standalone': 'false'
        }.items()
    ))

    return objects


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_bluefox_cam_and_imu',
            default_value='true',
            description='If you are running the thing in the separate Docker containers on the drone, set this to false, so only the vio is running in its own container'
        ),
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='false',
            description='If you are running the thing in the separate Docker containers on the drone, set this to false, so only the vio is running in its own container'
        ),
        DeclareLaunchArgument(
            'custom_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('mrs_open_vins_core'),
                'config',
                'realworld_bluefox_front',
                'custom_config_exmaple.yaml'
            ]),
            description='config from the user'
        ),
        OpaqueFunction(function=get_processed_launch_objects),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_rviz')),
            # Optional: specify a config file
            # arguments=['-d', os.path.join(get_package_share_directory('your_package'), 'config', 'your_config.rviz')]
        )
    ])