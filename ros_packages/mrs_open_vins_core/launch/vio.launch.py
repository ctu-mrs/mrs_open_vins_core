from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('bluefox2'),
                    'launch',
                    'single.launch.py'
                ])
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('mrs_serial'),
                    'launch',
                    'vio_imu.launch.py'
                ])
            ]),
        ),
        DeclareLaunchArgument(name='namespace',         default_value='',           description='namespace'),
        DeclareLaunchArgument(name='config',            default_value='realworld_bluefox_front',  description=''),
        DeclareLaunchArgument(name='verbosity',         default_value='DEBUG',       description='ALL, DEBUG, INFO, WARNING, ERROR, SILENT'),
        DeclareLaunchArgument(name='use_stereo',        default_value='false',       description=''),
        DeclareLaunchArgument(name='max_cameras',       default_value='1',          description=''),
        Node(package = 'ov_msckf',
            executable = 'run_subscribe_msckf',
            namespace = LaunchConfiguration('namespace'),
            parameters =[{'verbosity': LaunchConfiguration('verbosity')},
                        #{'use_stereo': LaunchConfiguration('use_stereo')},
                        #{'max_cameras': LaunchConfiguration('max_cameras')},
                        {'config_path': PathJoinSubstitution([
                                            FindPackageShare('mrs_open_vins_core'),
                                            'config',
                                            LaunchConfiguration('config'),
                                            'estimator_config.yaml'
                                        ])
                        },
                        {'topic_imu', '/imu_raw'},
                        {'topic_camera', '/uav1/image_raw'}
                    ],
            remappings=[('/cam0/image_raw', '/uav1/image_raw'),
                        ('/imu0', '/imu_raw')]
            #prefix="xterm -e gdb -ex=r --args",
            #prefix="gdb -ex=r --args",
        )
    ])