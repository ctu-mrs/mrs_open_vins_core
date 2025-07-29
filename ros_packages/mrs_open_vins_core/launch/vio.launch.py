import yaml
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
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
    
    
    objects.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bluefox2'),
                'launch',
                'single.launch.py'
            ])
        ]),
        launch_arguments={
            'custom_config': LaunchConfiguration('custom_config')
        }.items()
    ))
    
    objects.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mrs_serial'),
                'launch',
                'vio_imu.launch.py'
            ])
        ])
    ))
    
    objects.append(DeclareLaunchArgument(name='uav_name',          default_value=os.environ["UAV_NAME"]))
    objects.append(DeclareLaunchArgument(name='config',            default_value='realworld_bluefox_front',  description=''))
    objects.append(DeclareLaunchArgument(name='verbosity',         default_value='DEBUG',       description='ALL, DEBUG, INFO, WARNING, ERROR, SILENT'))
    objects.append(Node(package = 'ov_msckf',
        executable = executable_name,
        namespace = LaunchConfiguration('uav_name'),
        parameters =[{'verbosity': LaunchConfiguration('verbosity')},
                    {'config_path': PathJoinSubstitution([
                                        FindPackageShare('mrs_open_vins_core'),
                                        'config',
                                        LaunchConfiguration('config'),
                                        'estimator_config.yaml'
                                    ])
                    },
                    _custom_config_file
                ],
        #remappings=remappings,
        remappings=[('/imu_raw', '/vio_imu/imu_filtered')]
        #prefix="xterm -e gdb -ex=r --args",
        #prefix="gdb -ex=r --args",
    ))

    return objects


def generate_launch_description():
    return LaunchDescription([
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
        OpaqueFunction(function=get_processed_launch_objects)
    ])