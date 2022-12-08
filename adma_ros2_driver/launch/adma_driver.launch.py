from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():

        driver_config = PathJoinSubstitution([FindPackageShare('adma_ros2_driver'), 'config', 'driver_config.yaml'])
        driver_config_file = os.path.join(get_package_share_directory('adma_ros2_driver') , 'config', 'driver_config.yaml')
        with open(driver_config_file) as f:
                parameters = yaml.load(f, Loader=yaml.FullLoader)

        performance_check_arg = DeclareLaunchArgument('use_performance_check', default_value='True', description='True if timing check is required')
        driver_config_arg = DeclareLaunchArgument('driver_config', default_value=driver_config)
        log_level_arg = DeclareLaunchArgument('log_level', default_value='INFO')
        record_raw_data_arg = DeclareLaunchArgument('record_raw_data', default_value=str(parameters['genesys']['adma_ros2_driver']['ros__parameters']['record_raw_data']))

        adma_driver = Node(
                package='adma_ros2_driver',
                executable='adma_driver',
                output='screen',
                namespace='genesys',
                name='adma_ros2_driver',
                parameters=[LaunchConfiguration('driver_config')],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )

        rosbag_recorder = ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '/genesys/adma/data_recorded'],
                output='screen',
                condition=IfCondition(LaunchConfiguration('record_raw_data'))
        )
        
        return LaunchDescription([
                # # args
                performance_check_arg,
                driver_config_arg,
                log_level_arg,
                record_raw_data_arg,
                # #  nodes
                adma_driver,
                rosbag_recorder
        ])