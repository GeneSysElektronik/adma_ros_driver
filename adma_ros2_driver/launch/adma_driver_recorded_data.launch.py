from launch import LaunchDescription
from launch.substitutions import TextSubstitution, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, Shutdown
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

        driver_config = PathJoinSubstitution([FindPackageShare('adma_ros2_driver'), 'config', 'driver_config.yaml'])

        performance_check_arg = DeclareLaunchArgument('use_performance_check', default_value='True', description='True if timing check is required')
        driver_config_arg = DeclareLaunchArgument('driver_config', default_value=driver_config)
        log_level_arg = DeclareLaunchArgument('log_level', default_value='INFO')

        rosbag_file_arg = DeclareLaunchArgument('rosbag_file', default_value='/home/rschilli/ros2_ws/genesys/rosbag2_2022_12_08-11_00_02/')

        adma_driver = Node(
                package='adma_ros2_driver',
                executable='adma_driver',
                output='screen',
                namespace='genesys',
                name='adma_ros2_driver',
                parameters=[LaunchConfiguration('driver_config')],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )

        rosbag_player = ExecuteProcess(
                cmd=['ros2', 'bag', 'play', LaunchConfiguration('rosbag_file')],
                output='screen',
                on_exit=[LogInfo(msg=["Rosbag replay done. Stopping everything..."]),
                Shutdown(reason='launch is shutting down')],
        )
        
        return LaunchDescription([
                # # args
                performance_check_arg,
                driver_config_arg,
                log_level_arg,
                rosbag_file_arg,
                # #  nodes
                adma_driver,
                rosbag_player
        ])