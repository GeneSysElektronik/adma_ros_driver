from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

        # read default params
        driver_config = PathJoinSubstitution([FindPackageShare('adma_ros2_driver'), 'config', 'driver_config.yaml'])
        driver_config_arg = DeclareLaunchArgument('driver_config', default_value=driver_config)
        log_level_arg = DeclareLaunchArgument('log_level', default_value='INFO')
        rosbag_file_arg = DeclareLaunchArgument('rosbag_path', default_value='./')

        ### parameter for GSDB logging, used for ADMA-PP ####
        log_gsdb_arg = DeclareLaunchArgument('log_gsdb', default_value='True')
        raw_data_topic = '/genesys/adma/data_raw'

        ### parameters for recording data into a rosbag ###
        record_ros_bag_arg = DeclareLaunchArgument('record_rosbag', default_value='False')
        # list of desired topic to record. just comment/uncomment the entries you need
        recorded_topics = [
                # '/genesys/adma/data_raw', # unnecessary since its redundant logged in gsdb 
                # '/genesys/adma/data', # v3.3.3
                '/genesys/adma/data_scaled', # v3.3.5
                '/genesys/adma/status',
                '/genesys/adma/fix',
                '/genesys/adma/imu',
                '/genesys/adma/heading',
                '/genesys/adma/velocity'
        ]

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
                cmd=['ros2', 'bag', 'record', *recorded_topics],
                output='screen',
                condition=IfCondition(LaunchConfiguration('record_rosbag'))
        )

        gsdb_logger = Node(
                package='adma_tools_cpp',
                executable='bag2gsdb_converter',
                output='screen',
                namespace='genesys',
                name='bag2gsdb',
                parameters=[{
                        'rosbag_path': LaunchConfiguration('rosbag_path'),
                }],
                remappings=[
                        ("adma/data_recorded", raw_data_topic)
                ],
                condition=IfCondition(LaunchConfiguration('log_gsdb'))
        )
        
        return LaunchDescription([
                # # args
                driver_config_arg,
                log_level_arg,
                record_ros_bag_arg,
                rosbag_file_arg,
                log_gsdb_arg,
                # #  nodes
                adma_driver,
                rosbag_recorder,
                gsdb_logger
        ])