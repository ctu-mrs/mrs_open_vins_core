import yaml
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution, IfElseSubstitution, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode

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
    # Construct the container ID
    objects.append(DeclareLaunchArgument(name='container_name',         default_value='vio_main_container'))
    objects.append(DeclareLaunchArgument(name='container_namespace',    default_value=LaunchConfiguration('uav_name')))
    objects.append(DeclareLaunchArgument(name='container_id',           default_value=PathJoinSubstitution([LaunchConfiguration('container_namespace'), LaunchConfiguration('container_name')])))
    
    pkg_name = "mrs_uav_flightforge_simulator"
    pkg_share_path = get_package_share_directory(pkg_name)
    namespace='flightforge_simulator'
    mrs_simulator_path = get_package_share_directory("mrs_multirotor_simulator")
    
    custom_config = LaunchConfiguration('custom_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    # objects.append(DeclareLaunchArgument(
    #     'custom_config',
    #     default_value="",
    #     description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
    # ))

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
        condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
        if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
        else_value=custom_config
    )
    
    config_files = [
        # general configs
        pkg_share_path + '/config/flightforge_simulator.yaml',
        # uavs to load
        pkg_share_path + '/config/uavs.yaml',
    ]
    
    mrs_multirotor_simulator_uav_configs = [
        # general configs
        pkg_share_path + '/config/flightforge_simulator.yaml',
        # uavs to load
        pkg_share_path + '/config/uavs.yaml',
        # general configs
        mrs_simulator_path + '/config/controllers/attitude_controller.yaml',
        mrs_simulator_path + '/config/controllers/rate_controller.yaml',
        mrs_simulator_path + '/config/controllers/position_controller.yaml',
        mrs_simulator_path + '/config/controllers/velocity_controller.yaml',
        mrs_simulator_path + '/config/controllers/mixer.yaml',
        # the UAV configs
        mrs_simulator_path + '/config/uavs/a300.yaml',
        mrs_simulator_path + '/config/uavs/f330.yaml',
        mrs_simulator_path + '/config/uavs/f450.yaml',
        mrs_simulator_path + '/config/uavs/f550.yaml',
        mrs_simulator_path + '/config/uavs/naki.yaml',
        mrs_simulator_path + '/config/uavs/robofly.yaml',
        mrs_simulator_path + '/config/uavs/t650.yaml',
        mrs_simulator_path + '/config/uavs/x500.yaml',
    ]
    
    ff_wrapper = ComposableNode(
        package=pkg_name,
        plugin='mrs_uav_flightforge_simulator::FlightforgeSimulator',
        namespace='',
        name='flightforge_simulator',
        parameters=[
            {'config_files': config_files},
            {'custom_config': custom_config},
            {'uav_configs': mrs_multirotor_simulator_uav_configs},
            {'use_sim_time': True},
        ],

        remappings=[
            ("~/clock_out", "/clock"),
            ("~/uav_poses_out", "~/uav_poses"),
            ("/uav1/rgb/image_raw", "/uav1/image_raw"),
            ("/flightforge_simulator/uav1/imu", "/uav1/imu_filtered"),
            ("~/camera_info", PathJoinSubstitution(["/", LaunchConfiguration('uav_name'), "camera_info"]))
        ],
    )
    
    objects.append(ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace=LaunchConfiguration('container_namespace'),
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[ff_wrapper],
        #composable_node_descriptions=[],
        output='screen',
        #prefix='xterm -e gdb -ex run --args'
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
    
    # objects.append(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('mrs_open_vins_core'),
    #             'launch',
    #             'rgb_to_mono.launch.py'
    #         ])
    #     ]),
    #     condition=IfCondition(LaunchConfiguration('enable_bluefox_cam_and_imu')),
    #     launch_arguments={
    #         #'custom_config': LaunchConfiguration('custom_config')
    #         'container_id': LaunchConfiguration('container_id'),
    #         'standalone': 'false',
    #     }.items(),
    # ))

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