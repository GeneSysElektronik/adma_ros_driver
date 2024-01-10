from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

        # read default params
        decoder_config = PathJoinSubstitution([FindPackageShare('adma_tools_cpp'), 'config', 'raw_bag_decoder_config.yaml'])
        decoder_config_arg = DeclareLaunchArgument('decoder_config', default_value=decoder_config)
        log_level_arg = DeclareLaunchArgument('log_level', default_value='INFO')

        raw_bag_decoder = Node(
                package='adma_tools_cpp',
                executable='raw_bag_decoder',
                output='screen',
                namespace='genesys',
                name='raw_bag_decoder',
                parameters=[LaunchConfiguration('decoder_config')],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                remappings=[
                        # left=from / right=to (so publish the origin left on the new right topic)
                        ("adma/data", "adma/data"),
                        ("adma/data_raw", "adma/data_raw"),
                        ("adma/data_scaled", "adma/data_scaled"),
                        ("adma/status", "adma/status"),
                        ("adma/fix", "adma/fix"),
                        ("adma/imu", "adma/imu"),
                        ("adma/heading", "adma/heading"),
                        ("adma/velocity", "adma/velocity")
                ]
        )
        
        return LaunchDescription([
                # # args
                decoder_config_arg,
                log_level_arg,
                # #  nodes
                raw_bag_decoder
        ])