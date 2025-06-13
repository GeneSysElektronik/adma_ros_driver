from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    # define the path to your rosbag folder here, it will create a .gsdb file next to the db3 file
    rosbag_path = PathJoinSubstitution('/home/$USER/$ROS2_WS/data')
    rosbag_path_arg = DeclareLaunchArgument('rosbag_path', default_value=rosbag_path)
    log_addon_delta_arg = DeclareLaunchArgument('log_addon_delta', default_value='False')
    adma_namespace_arg = DeclareLaunchArgument('adma_namespace', default_value='genesys')

    rosbag_replay_rate_arg = DeclareLaunchArgument('replay_rate', default_value='1')

    bag2gsdb_converter = Node(
        package='adma_tools_cpp',
        executable='bag2gsdb_converter',
        output='screen',
        namespace=LaunchConfiguration('adma_namespace'),
        name='bag2gsdb',
        parameters=[
            {
                'rosbag_path': LaunchConfiguration('rosbag_path'),
                'log_addon_delta': LaunchConfiguration('log_addon_delta'),
            }
        ],
        remappings=[
            # left=from / right=to (so publish the origin left on the new right topic)
            # ('adma/data_raw', 'adma/data_recorded'),
            # ('adma/delta_raw', 'adma/delta_recorded'),
        ],
    )

    rosbag_player = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'play',
            LaunchConfiguration('rosbag_path'),
            '--rate',
            LaunchConfiguration('replay_rate'),
        ],
        output='screen',
        on_exit=[
            LogInfo(msg=['Rosbag replay done. Stopping everything...']),
            Shutdown(reason='launch is shutting down'),
        ],
    )

    return LaunchDescription(
        [
            # # args
            rosbag_path_arg,
            rosbag_replay_rate_arg,
            log_addon_delta_arg,
            adma_namespace_arg,
            # #  nodes
            rosbag_player,
            bag2gsdb_converter,
        ]
    )
