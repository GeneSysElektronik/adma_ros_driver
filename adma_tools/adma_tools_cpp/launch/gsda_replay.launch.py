from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo, Shutdown, ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

        gsda_config = PathJoinSubstitution([FindPackageShare('adma_tools_cpp'), 'config', 'gsda_replay_config.yaml'])
        gsda_config_arg = DeclareLaunchArgument('gsda_config', default_value=gsda_config)
        log_level_arg = DeclareLaunchArgument('log_level', default_value='INFO')

        # list of desired topic to record. just comment/uncomment the entries you need
        recorded_topics = [
                '/genesys/adma/data_scaled',
                '/genesys/adma/status',
                '/genesys/adma/fix',
                '/genesys/adma/imu',
                '/genesys/adma/heading',
                '/genesys/adma/velocity'
        ]

        rosbag_recorder = ExecuteProcess(
                cmd=['ros2', 'bag', 'record', *recorded_topics],
                output='screen',
        )

        gsda_server = Node(
                package='adma_tools_cpp',
                executable='gsda_server',
                output='screen',
                namespace='genesys',
                name='gsda_server',
                parameters=[LaunchConfiguration('gsda_config')],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                on_exit=[LogInfo(msg=["GSDA replay done. Stopping everything..."]),
                Shutdown(reason='launch is shutting down')],
        )
        
        return LaunchDescription([
                # # args
                gsda_config_arg,
                log_level_arg,
                # #  nodes
                rosbag_recorder,
                gsda_server
        ])
