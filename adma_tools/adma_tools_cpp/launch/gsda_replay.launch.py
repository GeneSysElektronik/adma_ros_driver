from launch import LaunchDescription
from launch.substitutions import TextSubstitution, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, Shutdown, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
import yaml
def generate_launch_description():

        gsda_path = PathJoinSubstitution('/home/rschilli/Documents/GeneSys/raw_data_60seconds_out_for.gsda')
        gsda_path_arg = DeclareLaunchArgument('gsda_file', default_value=gsda_path)
        frequency_arg = DeclareLaunchArgument('frequency', default_value='100')
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
                name='gsdb_server',
                parameters=[
                {
                        'frequency': LaunchConfiguration('frequency'),
                        'gsda_file': LaunchConfiguration('gsda_file')
                }
                ],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                on_exit=[LogInfo(msg=["GSDB replay done. Stopping everything..."]),
                Shutdown(reason='launch is shutting down')],
        )
        
        return LaunchDescription([
                # # args
                frequency_arg,
                gsda_path_arg,
                log_level_arg,
                # #  nodes
                rosbag_recorder,
                gsda_server
        ])