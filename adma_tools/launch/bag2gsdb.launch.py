from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

        filename = PathJoinSubstitution('recorded_bag.gsdb')
        filename_arg = DeclareLaunchArgument('filename', default_value=filename)

        bag2gsdb_converter = Node(
                package='adma_tools',
                executable='bag2gsdb',
                output='screen',
                namespace='genesys',
                name='bag2gsdb',
                parameters=[{
                'filename': LaunchConfiguration('filename')
                }]
        )
        
        return LaunchDescription([
                # # args
                filename_arg,
                # #  nodes
                bag2gsdb_converter
        ])