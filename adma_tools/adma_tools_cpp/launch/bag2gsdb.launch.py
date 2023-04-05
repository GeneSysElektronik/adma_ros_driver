from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, Shutdown
from launch_ros.actions import Node

def generate_launch_description():

        # define the path to your rosbag folder here, it will create a .gsdb file next to the db3 file
        rosbag_path = PathJoinSubstitution('/home/rschilli/Documents/GeneSys/rosbags_ros2/ROS2_Arbeitsplatz')
        rosbag_path_arg = DeclareLaunchArgument('rosbag_path', default_value=rosbag_path)

        rosbag_replay_rate_arg = DeclareLaunchArgument('replay_rate', default_value='1')

        bag2gsdb_converter = Node(
                package='adma_tools_cpp',
                executable='bag2gsdb_converter',
                output='screen',
                namespace='genesys',
                name='bag2gsdb',
                parameters=[{
                'rosbag_path': LaunchConfiguration('rosbag_path')
                }],
                remappings=[
                        # left=from / right=to (so publish the origin left on the new right topic)
                        # ("adma/data_raw", "adma/data_recorded"),
                ]
        )

        rosbag_player = ExecuteProcess(
                cmd=['ros2', 'bag', 'play', LaunchConfiguration('rosbag_path'),  '--rate',LaunchConfiguration('replay_rate')],
                output='screen',
                on_exit=[LogInfo(msg=["Rosbag replay done. Stopping everything..."]),
                Shutdown(reason='launch is shutting down')],
        )
        
        return LaunchDescription([
                # # args
                rosbag_path_arg,
                rosbag_replay_rate_arg,
                # #  nodes
                rosbag_player, 
                bag2gsdb_converter
        ])