from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

        
        xml_file_path = PathJoinSubstitution([FindPackageShare('adma_tools_py'), 'config', 'ADMA_UDP-DataStream_DELTA11_v7.0_v30.5.1.0.xml'])
        msg_file_output_path = PathJoinSubstitution([FindPackageShare('adma_ros_driver_msgs'), 'msg', 'Delta1170.msg'])
        xml_file_path_arg = DeclareLaunchArgument('xml_file', default_value=xml_file_path)
        msg_file_output_path_arg = DeclareLaunchArgument('msg_output_file', default_value=msg_file_output_path)

        xml2msg_generator = Node(
                package='adma_tools_py',
                executable='xml2msg',
                output='screen',
                namespace='genesys',
                name='xml2msg',
                parameters=[{
                'xml_file': LaunchConfiguration('xml_file'),
                'msg_output_file': LaunchConfiguration('msg_output_file')
                }]
        )
        
        return LaunchDescription([
                # # args
                xml_file_path_arg,
                msg_file_output_path_arg,
                # #  nodes
                xml2msg_generator 
        ])