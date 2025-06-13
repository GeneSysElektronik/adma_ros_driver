from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # read default params
    driver_config = PathJoinSubstitution(
        [FindPackageShare('adma_ros2_delta_driver'), 'config', 'driver_config.yaml']
    )
    driver_config_arg = DeclareLaunchArgument('driver_config', default_value=driver_config)
    log_level_arg = DeclareLaunchArgument('log_level', default_value='INFO')
    adma_namespace_arg = DeclareLaunchArgument('adma_namespace', default_value='genesys')

    adma_delta_driver = Node(
        package='adma_ros2_delta_driver',
        executable='adma_delta_driver',
        output='screen',
        namespace=LaunchConfiguration('adma_namespace'),
        name='adma_ros2_delta_driver',
        parameters=[LaunchConfiguration('driver_config')],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription(
        [
            # # args
            driver_config_arg,
            log_level_arg,
            adma_namespace_arg,
            # #  nodes
            adma_delta_driver,
        ]
    )
