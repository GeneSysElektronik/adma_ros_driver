from launch import LaunchDescription
from launch.substitutions import TextSubstitution, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

        adma_ip_address = TextSubstitution(text="0.0.0.0")
        adma_port = TextSubstitution(text="3456")

        driver_config = PathJoinSubstitution([FindPackageShare('adma_ros2_driver'), 'config', 'driver_config.yaml'])

        adma_ip_arg = DeclareLaunchArgument('adma_ip_address', default_value=adma_ip_address, description='IP address of the ADMA system')
        adma_port_arg = DeclareLaunchArgument('adma_port', default_value=adma_port, description='Broadcast port of the ADMA system')
        performance_check_arg = DeclareLaunchArgument('use_performance_check', default_value='True', description='True if timing check is required')
        driver_config_arg = DeclareLaunchArgument('driver_config', default_value=driver_config)

        adma_driver = Node(
                package='adma_ros2_driver',
                executable='adma_driver',
                output='screen',
                namespace='genesys',
                name='adma_ros2_driver',
                parameters=[LaunchConfiguration('driver_config')]
        )
        
        return LaunchDescription([
                # # args
                adma_ip_arg,
                adma_port_arg,
                performance_check_arg,
                driver_config_arg,
                # #  nodes
                adma_driver
        ])