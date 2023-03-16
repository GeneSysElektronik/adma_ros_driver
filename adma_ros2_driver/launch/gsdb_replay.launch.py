from launch import LaunchDescription
from launch.substitutions import TextSubstitution, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
import yaml
def generate_launch_description():

        driver_config = PathJoinSubstitution([FindPackageShare('adma_ros2_driver'), 'config', 'gsdb_replay_config.yaml'])
        driver_config_arg = DeclareLaunchArgument('driver_config', default_value=driver_config)
        log_level_arg = DeclareLaunchArgument('log_level', default_value='INFO')
        # overwrite ROS arg here to prevent creating new gsdb file during replaying another one..
        log_gsdb_arg = DeclareLaunchArgument('log_gsdb', default_value='False')

        # define the path to your rosbag folder here, it will create a .gsdb file next to the db3 file
        gsdb_path = PathJoinSubstitution('/home/rschilli/Documents/GeneSys/rosbags_ros2/ROS2_Arbeitsplatz/raw_data.gsdb')
        gsdb_path_arg = DeclareLaunchArgument('gsdb_file', default_value=gsdb_path)

        # set this to true if you want to replay GSDB and create a rosbag of it
        record_rosbag_arg = DeclareLaunchArgument('record_rosbag', default_value='False')

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
                parameters=[{
                'gsdb_file': LaunchConfiguration('gsdb_file'),
                'frequency': 100,
                'protocol_version': 'v3.3.4'
                }],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                on_exit=[LogInfo(msg=["GSDB replay done. Stopping everything..."]),
                Shutdown(reason='launch is shutting down')],
        )
        
        return LaunchDescription([
                # # args
                driver_config_arg,
                log_level_arg,
                log_gsdb_arg,
                gsdb_path_arg,
                record_rosbag_arg,
                # #  nodes
                adma_driver,
                data_server
        ])