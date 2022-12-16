from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, Shutdown
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():

        # read default params
        driver_config = PathJoinSubstitution([FindPackageShare('adma_ros2_driver'), 'config', 'driver_config.yaml'])
        driver_config_arg = DeclareLaunchArgument('driver_config', default_value=driver_config)
        log_level_arg = DeclareLaunchArgument('log_level', default_value='INFO')

        # read specific params of previos loaded config file
        driver_config_file = os.path.join(get_package_share_directory('adma_ros2_driver') , 'config', 'driver_config.yaml')
        with open(driver_config_file) as f:
                parameters = yaml.load(f, Loader=yaml.FullLoader)
        # extract mode
        mode = parameters['genesys']['adma_ros2_driver']['ros__parameters']['mode']

        ### parameters for recording data into a rosbag ###
        record_raw_data_arg = DeclareLaunchArgument('record_mode', default_value='False')
        if mode == 'record':
                record_raw_data_arg = DeclareLaunchArgument('record_mode', default_value='True')

        ### parameter for replaying rosbags ###
        # If mode = replay then it will use this arg to start the rosbag play
        replay_arg = DeclareLaunchArgument('replay_mode', default_value='False')
        if mode == 'replay':
                replay_arg = DeclareLaunchArgument('replay_mode', default_value='True')
        rosbag_file_arg = DeclareLaunchArgument('rosbag_file', default_value='/home/rschilli/ros2_ws/genesys/rosbag2_2022_12_09-18_31_51/')
        

        adma_driver = Node(
                package='adma_ros2_driver',
                executable='adma_driver',
                output='screen',
                namespace='genesys',
                name='adma_ros2_driver',
                parameters=[LaunchConfiguration('driver_config')],
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

        rosbag_recorder = ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '/genesys/adma/data_recorded'],
                output='screen',
                condition=IfCondition(LaunchConfiguration('record_mode'))
        )

        rosbag_player = ExecuteProcess(
                cmd=['ros2', 'bag', 'play', LaunchConfiguration('rosbag_file')],
                output='screen',
                condition=IfCondition(LaunchConfiguration('replay_mode')),
                on_exit=[LogInfo(msg=["Rosbag replay done. Stopping everything..."]),
                Shutdown(reason='launch is shutting down')],
        )
        
        return LaunchDescription([
                # # args
                driver_config_arg,
                log_level_arg,
                record_raw_data_arg,
                replay_arg,
                rosbag_file_arg,
                # #  nodes
                adma_driver,
                rosbag_recorder,
                rosbag_player,
        ])