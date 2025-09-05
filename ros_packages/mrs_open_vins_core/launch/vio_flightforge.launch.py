import yaml
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution, IfElseSubstitution, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

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
    
    # #{ custom config

    custom_config = LaunchConfiguration('custom_config')

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
        condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
        if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
        else_value=custom_config
    )

    # #} end of custom config
    
    objects.append(DeclareLaunchArgument(name='verbosity',         default_value='DEBUG',       description='ALL, DEBUG, INFO, WARNING, ERROR, SILENT'))
    
    objects.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ov_msckf'),
                'launch',
                'subscribe_composable.launch.py'
            ])
        ]),
        launch_arguments={
            #'custom_config': LaunchConfiguration('custom_config')
            # 'container_id': LaunchConfiguration('container_id'),
            'standalone': 'true',
            'config_path': PathJoinSubstitution([
                FindPackageShare('mrs_open_vins_core'),
                'config/simulation_flightforge',
                'estimator_config.yaml'
            ]),
            'custom_config': _custom_config_file
        }.items(),
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
        OpaqueFunction(function=get_processed_launch_objects),
    ])
