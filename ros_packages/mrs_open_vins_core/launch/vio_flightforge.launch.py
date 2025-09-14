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

def generate_launch_description():
    objects = []
    
    objects.append(DeclareLaunchArgument(name='uav_name', default_value=os.environ["UAV_NAME"]))
    
    objects.append(DeclareLaunchArgument(
        'custom_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('mrs_open_vins_core'),
            'config',
            'realworld_bluefox_front',
            'custom_config_exmaple.yaml'
        ]),
        description='config from the user'
    ))
    
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
    
    objects.append(DeclareLaunchArgument(name='verbosity', default_value='DEBUG', description='ALL, DEBUG, INFO, WARNING, ERROR, SILENT'))
    
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
            'custom_config': custom_config,
            'use_sim_time': 'true'
        }.items(),
    ))

    return LaunchDescription(objects)
