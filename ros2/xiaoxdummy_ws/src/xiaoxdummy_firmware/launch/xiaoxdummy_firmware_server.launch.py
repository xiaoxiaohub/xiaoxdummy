from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('command_mode', default_value='2'),
        DeclareLaunchArgument('command_speed', default_value='180.0'),
        Node(
            package='xiaoxdummy_firmware',
            executable='xiaoxdummy_firmware_server',
            name='xiaoxdummy_firmware_server',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'command_mode': LaunchConfiguration('command_mode'),
                'command_speed': LaunchConfiguration('command_speed'),
            }],
        ),
    ])
