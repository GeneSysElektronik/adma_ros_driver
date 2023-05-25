from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():

        driver_config = PathJoinSubstitution([FindPackageShare('adma_tools_cpp'), 'config', 'gsdb_replay_config.yaml'])
        driver_config_arg = DeclareLaunchArgument('driver_config', default_value=driver_config)
        log_level_arg = DeclareLaunchArgument('log_level', default_value='INFO')
        # overwrite ROS arg here to prevent creating new gsdb file during replaying another one..
        log_gsdb_arg = DeclareLaunchArgument('log_gsdb', default_value='False')

        # set this to true if you want to replay GSDB and create a rosbag of it
        record_rosbag_arg = DeclareLaunchArgument('record_rosbag', default_value='True')

        adma_driver = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                        FindPackageShare('adma_ros2_driver'),
                        'launch',
                        'adma_driver.launch.py'
                ]))
        )

        gsdb_server = Node(
                package='adma_tools_cpp',
                executable='gsdb_server',
                output='screen',
                namespace='genesys',
                name='gsdb_server',
                parameters=[LaunchConfiguration('driver_config')],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                on_exit=[LogInfo(msg=["GSDB replay done. Stopping everything..."]),
                Shutdown(reason='launch is shutting down')],
        )
        
        return LaunchDescription([
                # # args
                driver_config_arg,
                log_level_arg,
                log_gsdb_arg,
                record_rosbag_arg,
                # #  nodes
                adma_driver,
                gsdb_server
        ])