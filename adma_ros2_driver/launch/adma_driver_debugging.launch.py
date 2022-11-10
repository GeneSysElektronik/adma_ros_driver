from launch import LaunchDescription
from launch.substitutions import TextSubstitution, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

        driver_config = PathJoinSubstitution([FindPackageShare('adma_ros2_driver'), 'config', 'driver_config_debug.yaml'])

        driver_config_arg = DeclareLaunchArgument('driver_config', default_value=driver_config)
        log_level_arg = DeclareLaunchArgument('log_level', default_value='INFO')

        adma_driver = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                        FindPackageShare('adma_ros2_driver'),
                        'launch',
                        'adma_driver.launch.py'
                ]))
        )

        data_server = Node(
                package='adma_ros2_driver',
                executable='data_server',
                output='screen',
                namespace='genesys',
                name='data_server',
                parameters=[LaunchConfiguration('driver_config')],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )
        
        return LaunchDescription([
                # # args
                driver_config_arg,
                log_level_arg,
                # #  nodes
                adma_driver,
                data_server
        ])